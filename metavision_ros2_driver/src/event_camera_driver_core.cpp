/**
 * @file event_camera_driver_core.cpp
 * @brief Implementation of session-synchronized event recording.
 */

#include "event_camera_driver_core.hpp"
#include <iostream>
#include <iomanip>
#include <sstream>

namespace event_camera_driver {

// --- EventHandler Implementation ---

EventHandler::EventHandler(const std::string& serial, const std::string& bias_file, int acc_time_us) {
  camera_ = Metavision::Camera::from_serial(serial);
  serial_number_ = serial;

  if (!bias_file.empty()) {
    camera_.biases().set_from_file(bias_file);
  }

  auto geom = camera_.geometry();
  frame_gen_ = std::make_unique<Metavision::CDFrameGenerator>(geom.width(), geom.height());
  frame_gen_->set_display_accumulation_time_us(acc_time_us);

  camera_.cd().add_callback([this](const Metavision::EventCD *begin, const Metavision::EventCD *end) {
    std::lock_guard<std::mutex> lock(frame_mtx_);
    frame_gen_->add_events(begin, end);
  });

  camera_.start();
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
  // Deterministic filename based on logical role
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
  std::vector<std::string> discovered_serials = Metavision::DeviceDiscovery::list();

  for (const auto& serial : discovered_serials) {
    std::shared_ptr<EventHandler> handler = nullptr;
    if (serial == left_serial) {
      handler = std::make_shared<EventHandler>(serial, bias_file, acc_time_us);
      left_camera_ = handler;
    } else if (serial == right_serial) {
      handler = std::make_shared<EventHandler>(serial, bias_file, acc_time_us);
      right_camera_ = handler;
    } else {
      handler = std::make_shared<EventHandler>(serial, bias_file, acc_time_us);
    }
    if (handler) all_handlers_.push_back(handler);
  }
}

// Accepts session_id from Master node
void EventDriverCore::start_all_recording(const std::string& session_id) {
  fs::path session_path = fs::path(base_save_dir_) / session_id;

  if (left_camera_) {
    left_camera_->start_recording((session_path / "event_left").string(), "left");
  }
  if (right_camera_) {
    right_camera_->start_recording((session_path / "event_right").string(), "right");
  }
  
  for (auto& h : all_handlers_) {
    if (h != left_camera_ && h != right_camera_) {
      h->start_recording((session_path / ("event_diag_" + h->get_serial())).string(), h->get_serial());
    }
  }
}

void EventDriverCore::stop_all_recording() {
  for (auto& h : all_handlers_) h->stop_recording();
}

bool EventDriverCore::is_any_recording() const {
  for (auto& h : all_handlers_) if (h->is_recording()) return true;
  return false;
}

std::vector<cv::Mat> EventDriverCore::get_all_frames(bool rotate_180) {
  std::vector<cv::Mat> frames;
  if (left_camera_) {
    auto f = left_camera_->get_latest_frame(rotate_180);
    if (!f.empty()) frames.push_back(f);
  }
  if (right_camera_) {
    auto f = right_camera_->get_latest_frame(rotate_180);
    if (!f.empty()) frames.push_back(f);
  }
  return frames;
}

} // namespace event_camera_driver