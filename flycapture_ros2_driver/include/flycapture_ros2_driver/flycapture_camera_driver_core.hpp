/**
 * @file flycapture_camera_driver_core.hpp
 * @brief Master controller for FlyCapture cameras with Session ID generation.
 * @copyright Copyright (c) 2026
 */

#ifndef FLYCAPTURE_CAMERA_DRIVER_CORE_HPP_
#define FLYCAPTURE_CAMERA_DRIVER_CORE_HPP_

#include <FlyCapture2.h>
#include <opencv2/opencv.hpp>
#include <filesystem>
#include <queue>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <memory>
#include <vector>
#include <string>

namespace flycapture_driver {

namespace fs = std::filesystem;

class CameraHandler {
public:
  struct ImageData {
    cv::Mat image;
    FlyCapture2::TimeStamp timestamp;
  };

  CameraHandler(unsigned int index, FlyCapture2::BusManager &bus_mgr,
                const std::string &video_mode, const std::string &hardware_frame_rate,
                int timeout_ms);
  ~CameraHandler();

  CameraHandler(const CameraHandler &) = delete;
  CameraHandler &operator=(const CameraHandler &) = delete;

  void grab_frame(cv::Mat &frame, FlyCapture2::TimeStamp &ts);
  void start_recording(const std::string &dir);
  void stop_recording();
  bool is_recording() const { return is_recording_; }
  unsigned int get_serial_number() const { return serial_number_; }

private:
  void save_worker();
  FlyCapture2::VideoMode parse_video_mode(const std::string &str);
  FlyCapture2::FrameRate parse_frame_rate(const std::string &str);

  unsigned int index_;
  unsigned int serial_number_;
  std::unique_ptr<FlyCapture2::Camera> camera_;
  std::string current_dir_;
  bool is_recording_ = false;
  bool stop_worker_ = false;

  std::queue<ImageData> queue_;
  std::mutex mtx_;
  std::condition_variable cv_;
  std::thread worker_thread_;
};

class DriverCore {
public:
  DriverCore();
  ~DriverCore() = default;

  void initialize_cameras(const std::string &right_serial, const std::string &left_serial,
                          const std::string &video_mode, const std::string &hardware_frame_rate,
                          int timeout, const std::string &base_dir);

  std::vector<cv::Mat> get_all_frames(bool rotate_180);
  
  // Revised: Accepts an externally generated session ID
  void start_all_recording(const std::string& session_id);
  void stop_all_recording();
  
  // Helper for Master Node to generate the common ID
  std::string generate_session_name();
  bool is_any_recording() const;

private:
  FlyCapture2::BusManager bus_mgr_;
  std::vector<std::shared_ptr<CameraHandler>> handlers_;
  std::shared_ptr<CameraHandler> left_camera_ = nullptr;
  std::shared_ptr<CameraHandler> right_camera_ = nullptr;
  std::vector<std::shared_ptr<CameraHandler>> random_cameras_;
  std::string base_save_dir_;
};

} // namespace flycapture_driver

#endif // FLYCAPTURE_CAMERA_DRIVER_CORE_HPP_