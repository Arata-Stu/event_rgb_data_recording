#include "flycapture_camera_driver_core.hpp"
#include <iomanip>
#include <sstream>
#include <ctime>
#include <chrono>

namespace flycapture_driver {

CameraHandler::CameraHandler(unsigned int index, FlyCapture2::BusManager &bus_mgr,
                             const std::string &video_mode, const std::string &hardware_frame_rate,
                             int timeout_ms)
    : index_(index), camera_(std::make_unique<FlyCapture2::Camera>()) {
  FlyCapture2::PGRGuid guid;
  bus_mgr.GetCameraFromIndex(index, &guid);
  camera_->Connect(&guid);
  FlyCapture2::CameraInfo info;
  camera_->GetCameraInfo(&info);
  serial_number_ = info.serialNumber;
  
  FlyCapture2::FC2Config config;
  camera_->GetConfiguration(&config);
  config.grabTimeout = timeout_ms;
  camera_->SetConfiguration(&config);
  camera_->SetVideoModeAndFrameRate(parse_video_mode(video_mode), parse_frame_rate(hardware_frame_rate));
  camera_->StartCapture();
  worker_thread_ = std::thread(&CameraHandler::save_worker, this);
}

CameraHandler::~CameraHandler() {
  { std::lock_guard<std::mutex> lock(mtx_); stop_worker_ = true; }
  cv_.notify_all();
  if (worker_thread_.joinable()) worker_thread_.join();
  camera_->StopCapture();
  camera_->Disconnect();
}

void CameraHandler::grab_frame(cv::Mat &frame, FlyCapture2::TimeStamp &ts) {
  FlyCapture2::Image raw;
  if (camera_->RetrieveBuffer(&raw) != FlyCapture2::PGRERROR_OK) { frame = cv::Mat(); return; }
  ts = raw.GetTimeStamp();
  FlyCapture2::Image bgr;
  raw.Convert(FlyCapture2::PIXEL_FORMAT_BGR, &bgr);
  frame = cv::Mat(bgr.GetRows(), bgr.GetCols(), CV_8UC3, bgr.GetData()).clone();
  if (is_recording_) {
    std::lock_guard<std::mutex> lock(mtx_);
    queue_.push({frame.clone(), ts});
    cv_.notify_one();
  }
}

void CameraHandler::start_recording(const std::string &dir) {
  fs::create_directories(dir);
  current_dir_ = dir;
  is_recording_ = true;
}

void CameraHandler::stop_recording() { is_recording_ = false; }

void CameraHandler::save_worker() {
  while (true) {
    ImageData data;
    {
      std::unique_lock<std::mutex> lock(mtx_);
      cv_.wait(lock, [this] { return !queue_.empty() || stop_worker_; });
      if (stop_worker_ && queue_.empty()) break;
      data = queue_.front();
      queue_.pop();
    }
    std::time_t t = data.timestamp.seconds;
    std::ostringstream ss;
    ss << current_dir_ << "/" << std::put_time(std::localtime(&t), "%Y%m%d_%H%M%S")
       << "_" << std::setw(6) << std::setfill('0') << data.timestamp.microSeconds << ".jpg";
    cv::imwrite(ss.str(), data.image, {cv::IMWRITE_JPEG_QUALITY, 90});
  }
}

FlyCapture2::VideoMode CameraHandler::parse_video_mode(const std::string &str) {
  if (str == "VIDEOMODE_800x600YUV422") return FlyCapture2::VIDEOMODE_800x600YUV422;
  return FlyCapture2::VIDEOMODE_FORMAT7;
}

FlyCapture2::FrameRate CameraHandler::parse_frame_rate(const std::string &str) {
  return FlyCapture2::FRAMERATE_30;
}

// --- DriverCore Implementation ---

DriverCore::DriverCore() {}

void DriverCore::initialize_cameras(const std::string &right_serial, const std::string &left_serial,
                                    const std::string &video_mode, const std::string &hardware_frame_rate,
                                    int timeout, const std::string &base_dir) {
  base_save_dir_ = base_dir;
  unsigned int num_cams;
  bus_mgr_.GetNumOfCameras(&num_cams);
  unsigned int target_r = right_serial.empty() ? 0 : std::stoul(right_serial);
  unsigned int target_l = left_serial.empty() ? 0 : std::stoul(left_serial);

  for (unsigned int i = 0; i < num_cams; ++i) {
    auto h = std::make_shared<CameraHandler>(i, bus_mgr_, video_mode, hardware_frame_rate, timeout);
    handlers_.push_back(h);
    if (h->get_serial_number() == target_r) right_camera_ = h;
    else if (h->get_serial_number() == target_l) left_camera_ = h;
    else random_cameras_.push_back(h);
  }
}

// Updated: Uses session_id to define directory structure
void DriverCore::start_all_recording(const std::string& session_id) {
  fs::path session_path = fs::path(base_save_dir_) / session_id;
  for (auto &h : handlers_) {
    std::string sub;
    if (h == left_camera_) sub = "left";
    else if (h == right_camera_) sub = "right";
    else sub = "cam_" + std::to_string(h->get_serial_number());
    h->start_recording((session_path / sub).string());
  }
}

void DriverCore::stop_all_recording() {
  for (auto &h : handlers_) h->stop_recording();
}

std::string DriverCore::generate_session_name() {
  auto now = std::chrono::system_clock::now();
  auto t = std::chrono::system_clock::to_time_t(now);
  auto us = std::chrono::duration_cast<std::chrono::microseconds>(now.time_since_epoch()).count() % 1000000;
  std::ostringstream ss;
  ss << std::put_time(std::localtime(&t), "%Y%m%d_%H%M%S") << "_" << std::setw(6) << std::setfill('0') << us;
  return ss.str();
}

bool DriverCore::is_any_recording() const {
  for (auto &h : handlers_) if (h->is_recording()) return true;
  return false;
}

std::vector<cv::Mat> DriverCore::get_all_frames(bool rotate_180) {
  std::vector<cv::Mat> frames;
  auto proc = [&](std::shared_ptr<CameraHandler> h) {
    if (!h) return;
    cv::Mat f; FlyCapture2::TimeStamp ts; h->grab_frame(f, ts);
    if (!f.empty()) { if (rotate_180) cv::rotate(f, f, cv::ROTATE_180); frames.push_back(f); }
  };
  proc(left_camera_); proc(right_camera_);
  for (auto &h : random_cameras_) proc(h);
  return frames;
}

} // namespace flycapture_driver#include "flycapture_camera_driver_core.hpp"
#include <iomanip>
#include <sstream>
#include <ctime>
#include <chrono>

namespace flycapture_driver {

// --- CameraHandler Implementation (Same as before) ---
// ... [Previous CameraHandler methods remain identical] ...

CameraHandler::CameraHandler(unsigned int index, FlyCapture2::BusManager &bus_mgr,
                             const std::string &video_mode, const std::string &hardware_frame_rate,
                             int timeout_ms)
    : index_(index), camera_(std::make_unique<FlyCapture2::Camera>()) {
  FlyCapture2::PGRGuid guid;
  bus_mgr.GetCameraFromIndex(index, &guid);
  camera_->Connect(&guid);
  FlyCapture2::CameraInfo info;
  camera_->GetCameraInfo(&info);
  serial_number_ = info.serialNumber;
  
  FlyCapture2::FC2Config config;
  camera_->GetConfiguration(&config);
  config.grabTimeout = timeout_ms;
  camera_->SetConfiguration(&config);
  camera_->SetVideoModeAndFrameRate(parse_video_mode(video_mode), parse_frame_rate(hardware_frame_rate));
  camera_->StartCapture();
  worker_thread_ = std::thread(&CameraHandler::save_worker, this);
}

CameraHandler::~CameraHandler() {
  { std::lock_guard<std::mutex> lock(mtx_); stop_worker_ = true; }
  cv_.notify_all();
  if (worker_thread_.joinable()) worker_thread_.join();
  camera_->StopCapture();
  camera_->Disconnect();
}

void CameraHandler::grab_frame(cv::Mat &frame, FlyCapture2::TimeStamp &ts) {
  FlyCapture2::Image raw;
  if (camera_->RetrieveBuffer(&raw) != FlyCapture2::PGRERROR_OK) { frame = cv::Mat(); return; }
  ts = raw.GetTimeStamp();
  FlyCapture2::Image bgr;
  raw.Convert(FlyCapture2::PIXEL_FORMAT_BGR, &bgr);
  frame = cv::Mat(bgr.GetRows(), bgr.GetCols(), CV_8UC3, bgr.GetData()).clone();
  if (is_recording_) {
    std::lock_guard<std::mutex> lock(mtx_);
    queue_.push({frame.clone(), ts});
    cv_.notify_one();
  }
}

void CameraHandler::start_recording(const std::string &dir) {
  fs::create_directories(dir);
  current_dir_ = dir;
  is_recording_ = true;
}

void CameraHandler::stop_recording() { is_recording_ = false; }

void CameraHandler::save_worker() {
  while (true) {
    ImageData data;
    {
      std::unique_lock<std::mutex> lock(mtx_);
      cv_.wait(lock, [this] { return !queue_.empty() || stop_worker_; });
      if (stop_worker_ && queue_.empty()) break;
      data = queue_.front();
      queue_.pop();
    }
    std::time_t t = data.timestamp.seconds;
    std::ostringstream ss;
    ss << current_dir_ << "/" << std::put_time(std::localtime(&t), "%Y%m%d_%H%M%S")
       << "_" << std::setw(6) << std::setfill('0') << data.timestamp.microSeconds << ".jpg";
    cv::imwrite(ss.str(), data.image, {cv::IMWRITE_JPEG_QUALITY, 90});
  }
}

FlyCapture2::VideoMode CameraHandler::parse_video_mode(const std::string &str) {
  if (str == "VIDEOMODE_800x600YUV422") return FlyCapture2::VIDEOMODE_800x600YUV422;
  return FlyCapture2::VIDEOMODE_FORMAT7;
}

FlyCapture2::FrameRate CameraHandler::parse_frame_rate(const std::string &str) {
  return FlyCapture2::FRAMERATE_30;
}

// --- DriverCore Implementation ---

DriverCore::DriverCore() {}

void DriverCore::initialize_cameras(const std::string &right_serial, const std::string &left_serial,
                                    const std::string &video_mode, const std::string &hardware_frame_rate,
                                    int timeout, const std::string &base_dir) {
  base_save_dir_ = base_dir;
  unsigned int num_cams;
  bus_mgr_.GetNumOfCameras(&num_cams);
  unsigned int target_r = right_serial.empty() ? 0 : std::stoul(right_serial);
  unsigned int target_l = left_serial.empty() ? 0 : std::stoul(left_serial);

  for (unsigned int i = 0; i < num_cams; ++i) {
    auto h = std::make_shared<CameraHandler>(i, bus_mgr_, video_mode, hardware_frame_rate, timeout);
    handlers_.push_back(h);
    if (h->get_serial_number() == target_r) right_camera_ = h;
    else if (h->get_serial_number() == target_l) left_camera_ = h;
    else random_cameras_.push_back(h);
  }
}

// Updated: Uses session_id to define directory structure
void DriverCore::start_all_recording(const std::string& session_id) {
  fs::path session_path = fs::path(base_save_dir_) / session_id;
  for (auto &h : handlers_) {
    std::string sub;
    if (h == left_camera_) sub = "left";
    else if (h == right_camera_) sub = "right";
    else sub = "cam_" + std::to_string(h->get_serial_number());
    h->start_recording((session_path / sub).string());
  }
}

void DriverCore::stop_all_recording() {
  for (auto &h : handlers_) h->stop_recording();
}

std::string DriverCore::generate_session_name() {
  auto now = std::chrono::system_clock::now();
  auto t = std::chrono::system_clock::to_time_t(now);
  auto us = std::chrono::duration_cast<std::chrono::microseconds>(now.time_since_epoch()).count() % 1000000;
  std::ostringstream ss;
  ss << std::put_time(std::localtime(&t), "%Y%m%d_%H%M%S") << "_" << std::setw(6) << std::setfill('0') << us;
  return ss.str();
}

bool DriverCore::is_any_recording() const {
  for (auto &h : handlers_) if (h->is_recording()) return true;
  return false;
}

std::vector<cv::Mat> DriverCore::get_all_frames(bool rotate_180) {
  std::vector<cv::Mat> frames;
  auto proc = [&](std::shared_ptr<CameraHandler> h) {
    if (!h) return;
    cv::Mat f; FlyCapture2::TimeStamp ts; h->grab_frame(f, ts);
    if (!f.empty()) { if (rotate_180) cv::rotate(f, f, cv::ROTATE_180); frames.push_back(f); }
  };
  proc(left_camera_); proc(right_camera_);
  for (auto &h : random_cameras_) proc(h);
  return frames;
}

} // namespace flycapture_driver