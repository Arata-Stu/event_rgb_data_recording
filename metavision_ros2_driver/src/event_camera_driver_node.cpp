/**
 * @file event_camera_driver_node.cpp
 * @brief Follower Node that parses Master Session IDs for directory synchronization.
 */

#include "event_camera_driver_core.hpp"
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

using namespace event_camera_driver;

class EventCameraNode : public rclcpp::Node {
public:
  EventCameraNode() : Node("event_camera_driver_node") {
    // --- Parameters ---
    std::string l_sn = this->declare_parameter<std::string>("left_event_serial", "");
    std::string r_sn = this->declare_parameter<std::string>("right_event_serial", "");
    std::string bias_file = this->declare_parameter<std::string>("bias_file", "");
    save_directory_ = this->declare_parameter<std::string>("save_directory", "/tmp/recording");
    
    int timer_ms = this->declare_parameter<int>("timer_period_ms", 33);
    int acc_us = this->declare_parameter<int>("accumulation_time_us", 10000);
    rotate_180_ = this->declare_parameter<bool>("rotate_180", false);
    show_window_ = this->declare_parameter<bool>("show_window", true);

    // --- Core Setup ---
    core_ = std::make_unique<EventDriverCore>();
    core_->initialize_cameras(l_sn, r_sn, bias_file, acc_us, save_directory_);

    // --- ROS Interfaces ---
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(timer_ms),
        std::bind(&EventCameraNode::on_timer, this));

    // Subscribe to status topic to receive Session ID
    status_sub_ = this->create_subscription<std_msgs::msg::String>(
        "recording_status", 10,
        std::bind(&EventCameraNode::on_status_msg, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Stereo Event Camera (Follower) Node Ready.");
  }

private:
  void on_timer() {
    auto frames = core_->get_all_frames(rotate_180_);
    if (!frames.empty() && show_window_) {
      cv::Mat combined;
      cv::hconcat(frames, combined);
      cv::imshow("Stereo Event Camera Preview", combined);
      cv::waitKey(1);
    }
  }

  /**
   * @brief Parses the command and extracts the Session ID.
   * Format expected: "start:YYYYMMDD_HHMMSS_uuuuuu" or "switch:..."
   */
  void on_status_msg(const std_msgs::msg::String::SharedPtr msg) {
    std::string data = msg->data;

    if (data.find("start:") == 0) {
      std::string session_id = data.substr(6);
      core_->start_all_recording(session_id);
      RCLCPP_INFO(this->get_logger(), "Session Sync Start: %s", session_id.c_str());

    } else if (data.find("switch:") == 0) {
      std::string session_id = data.substr(7);
      core_->stop_all_recording();
      core_->start_all_recording(session_id);
      RCLCPP_INFO(this->get_logger(), "Session Sync Switch: %s", session_id.c_str());

    } else if (data == "stop") {
      core_->stop_all_recording();
      RCLCPP_INFO(this->get_logger(), "Session Sync Stop.");
    }
  }

  std::unique_ptr<EventDriverCore> core_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr status_sub_;
  
  std::string save_directory_;
  bool rotate_180_;
  bool show_window_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<EventCameraNode>());
  rclcpp::shutdown();
  return 0;
}