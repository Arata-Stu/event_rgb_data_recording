/**
 * @file event_camera_driver_core.cpp
 * @brief Implementation of hardware discovery and role-based camera management.
 */

#include "event_camera_driver_core.hpp"
#include <iostream>
#include <iomanip>
#include <sstream>
#include <ctime>
#include <chrono>

namespace event_camera_driver {

// --- EventHandler Implementation ---

EventHandler::EventHandler(const std::string& serial, const std::string& bias_file, int acc_time_us) {
  // Initialize camera using exact serial number
  camera_ = Metavision::Camera::from_serial(serial);
  serial_number_ = serial;

  if (!bias_file.empty()) {
    camera_.biases().set_from_file(bias_file);
  }

  auto geom = camera_.geometry();
  frame_gen_ = std::make_unique<Metavision::CDFrameGenerator>(geom.width(), geom.height());
  frame_gen_->set_display_accumulation_time_us(acc_time_us);

  // Register CD callback for frame generation
  camera_.cd().add_callback([this](const Metavision::EventCD *begin, const Metavision::EventCD *end) {
    std::lock_guard<std::mutex> lock(frame_mtx_);
    frame_gen_->add_events(begin, end);
  });

  camera_.start();
  
  // Start the frame generation thread (30 FPS refresh)
  frame_gen_->start(30, [this](Metavision::timestamp, const cv::Mat &frame) {
    std::lock_guard<std::mutex> lock(frame_mtx_);
    frame.copyTo(current_frame_);
  });
}

EventHandler::~EventHandler() {
  if (is_recording_) stop_recording();
  if (camera_.is_running()) camera_.stop();
}

void EventHandler::start_recording(const std::string& dir_path, const std::string& role_name) {
  fs::create_directories(dir_path);
  // Deterministic filename based on role (left.raw / right.raw)
  std::string file_path = (fs::path(dir_path) / (role_name + ".raw")).string();
  camera_.start_recording(file_path);
  is_recording_ = true;
}

void EventHandler::stop_recording() {
  camera_.stop_recording();
  is_recording_ = false;
}

cv::Mat EventHandler::get_latest_frame(bool rotate_180) {
  std::lock_guard<std::mutex> lock(frame_mtx_);
  if (current_frame_.empty()) return cv::Mat();

  cv::Mat output = current_frame_.clone();
  if (rotate_180) cv::rotate(output, output, cv::ROTATE_180);
  return output;
}

// --- EventDriverCore Implementation ---

EventDriverCore::EventDriverCore() {}

void EventDriverCore::initialize_cameras(const std::string& left_serial, const std::string& right_serial,
                                         const std::string& bias_file, int acc_time_us, 
                                         const std::string& base_dir) {
  base_save_dir_ = base_dir;

  // Perform Hardware Discovery
  std::vector<std::string> discovered_serials = Metavision::DeviceDiscovery::list();
  if (discovered_serials.empty()) {
    throw std::runtime_error("Metavision SDK: No cameras detected on USB bus.");
  }

  for (const auto& serial : discovered_serials) {
    std::shared_ptr<EventHandler> handler = nullptr;

    // Role Assignment Logic
    if (serial == left_serial) {
      handler = std::make_shared<EventHandler>(serial, bias_file, acc_time_us);
      left_camera_ = handler;
    } else if (serial == right_serial) {
      handler = std::make_shared<EventHandler>(serial, bias_file, acc_time_us);
      right_camera_ = handler;
    } else {
      // Diagnostic camera initialization
      handler = std::make_shared<EventHandler>(serial, bias_file, acc_time_us);
    }

    if (handler) all_handlers_.push_back(handler);
  }
}

std::string EventDriverCore::start_all_recording() {
  std::string session_name = generate_session_name();
  fs::path session_path = fs::path(base_save_dir_) / session_name;

  if (left_camera_) {
    left_camera_->start_recording((session_path / "event_left").string(), "left");
  }
  if (right_camera_) {
    right_camera_->start_recording((session_path / "event_right").string(), "right");
  }
  
  // Record unassigned cameras with serial-based folder names
  for (auto& h : all_handlers_) {
    if (h != left_camera_ && h != right_camera_) {
      h->start_recording((session_path / ("event_diag_" + h->get_serial())).string(), h->get_serial());
    }
  }
  return session_path.string();
}

void EventDriverCore::stop_all_recording() {
  for (auto& h : all_handlers_) h->stop_recording();
}

std::string EventDriverCore::switch_all_directories() {
  if (!is_any_recording()) return "";
  stop_all_recording();
  return start_all_recording();
}

bool EventDriverCore::is_any_recording() const {
  for (auto& h : all_handlers_) if (h->is_recording()) return true;
  return false;
}

std::vector<cv::Mat> EventDriverCore::get_all_frames(bool rotate_180) {
  std::vector<cv::Mat> frames;
  if (left_camera_) frames.push_back(left_camera_->get_latest_frame(rotate_180));
  if (right_camera_) frames.push_back(right_camera_->get_latest_frame(rotate_180));
  return frames;
}

std::string EventDriverCore::generate_session_name() {
  auto now = std::chrono::system_clock::now();
  auto t = std::chrono::system_clock::to_time_t(now);
  auto us = std::chrono::duration_cast<std::chrono::microseconds>(now.time_since_epoch()).count() % 1000000;
  
  std::ostringstream ss;
  ss << std::put_time(std::localtime(&t), "%Y%m%d_%H%M%S") << "_" << std::setw(6) << std::setfill('0') << us;
  return ss.str();
}

} // namespace event_camera_driver