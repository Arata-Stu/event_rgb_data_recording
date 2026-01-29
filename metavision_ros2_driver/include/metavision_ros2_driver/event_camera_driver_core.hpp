/**
 * @file event_camera_driver_core.hpp
 * @brief Follower controller for Event Cameras that synchronizes with Master Session IDs.
 * @copyright Copyright (c) 2026
 */

#ifndef EVENT_CAMERA_DRIVER_CORE_HPP_
#define EVENT_CAMERA_DRIVER_CORE_HPP_

#include <metavision/sdk/driver/camera.h>
#include <metavision/sdk/driver/device_discovery.h>
#include <metavision/sdk/core/utils/cd_frame_generator.h>
#include <opencv2/opencv.hpp>
#include <filesystem>
#include <mutex>
#include <memory>
#include <vector>
#include <string>

namespace event_camera_driver {

namespace fs = std::filesystem;

class EventHandler {
public:
  EventHandler(const std::string& serial, const std::string& bias_file, int acc_time_us);
  ~EventHandler();

  EventHandler(const EventHandler&) = delete;
  EventHandler& operator=(const EventHandler&) = delete;

  /**
   * @brief Starts recording to a path derived from the Master Session ID.
   */
  void start_recording(const std::string& dir_path, const std::string& role_name);
  void stop_recording();
  
  cv::Mat get_latest_frame(bool rotate_180);
  bool is_recording() const { return is_recording_; }
  std::string get_serial() const { return serial_number_; }

private:
  Metavision::Camera camera_;
  std::unique_ptr<Metavision::CDFrameGenerator> frame_gen_;
  cv::Mat current_frame_;
  std::mutex frame_mtx_;
  std::string serial_number_;
  bool is_recording_ = false;
};

class EventDriverCore {
public:
  EventDriverCore();
  ~EventDriverCore() = default;

  void initialize_cameras(const std::string& left_serial, const std::string& right_serial,
                          const std::string& bias_file, int acc_time_us, 
                          const std::string& base_dir);

  std::vector<cv::Mat> get_all_frames(bool rotate_180);

  // Revised: Accepts the session_id from the Master node via ROS topic
  void start_all_recording(const std::string& session_id);
  void stop_all_recording();
  
  bool is_any_recording() const;

private:
  std::shared_ptr<EventHandler> left_camera_ = nullptr;
  std::shared_ptr<EventHandler> right_camera_ = nullptr;
  std::vector<std::shared_ptr<EventHandler>> all_handlers_;
  
  std::string base_save_dir_;
};

} // namespace event_camera_driver

#endif // EVENT_CAMERA_DRIVER_CORE_HPP_