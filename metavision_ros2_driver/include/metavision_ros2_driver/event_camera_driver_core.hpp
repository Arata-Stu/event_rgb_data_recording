/**
 * @file event_camera_driver_core.hpp
 * @brief Core logic for managing multiple Metavision Event Cameras.
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

/**
 * @class EventHandler
 * @brief Encapsulates a single Metavision camera instance.
 * Handles event-to-frame generation and RAW recording for a specific serial.
 */
class EventHandler {
public:
  EventHandler(const std::string& serial, const std::string& bias_file, int acc_time_us);
  ~EventHandler();

  // Disable copy/assignment to prevent double-resource handling
  EventHandler(const EventHandler&) = delete;
  EventHandler& operator=(const EventHandler&) = delete;

  /**
   * @brief Starts recording to a specific path.
   * @param dir_path Directory where the file will be saved.
   * @param role_name Name of the role (e.g., "left", "right") used for the filename.
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

/**
 * @class EventDriverCore
 * @brief Orchestrates multiple EventHandler instances based on hardware discovery.
 */
class EventDriverCore {
public:
  EventDriverCore();
  ~EventDriverCore() = default;

  /**
   * @brief Discovers connected cameras and assigns roles based on serial numbers.
   */
  void initialize_cameras(const std::string& left_serial, const std::string& right_serial,
                          const std::string& bias_file, int acc_time_us, 
                          const std::string& base_dir);

  std::vector<cv::Mat> get_all_frames(bool rotate_180);
  std::string start_all_recording();
  void stop_all_recording();
  std::string switch_all_directories();
  bool is_any_recording() const;

private:
  std::string generate_session_name();

  std::shared_ptr<EventHandler> left_camera_ = nullptr;
  std::shared_ptr<EventHandler> right_camera_ = nullptr;
  std::vector<std::shared_ptr<EventHandler>> all_handlers_;
  
  std::string base_save_dir_;
};

} // namespace event_camera_driver

#endif // EVENT_CAMERA_DRIVER_CORE_HPP_