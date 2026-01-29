/**
 * @file event_camera_driver_node.cpp
 * @brief ROS 2 Node wrapper for stereo event camera management.
 */

#include "event_camera_driver_core.hpp"
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

using namespace event_camera_driver;

class EventCameraNode : public rclcpp::Node {
public:
  EventCameraNode() : Node("event_camera_driver_node") {
    // --- Parameter Declaration ---
    // User-provided serials to map physical hardware to logical roles
    std::string left_sn = this->declare_parameter<std::string>("left_event_serial", "");
    std::string right_sn = this->declare_parameter<std::string>("right_event_serial", "");
    std::string bias_file = this->declare_parameter<std::string>("bias_file", "");
    
    // Persistent configuration members
    save_directory_ = this->declare_parameter<std::string>("save_directory", "/tmp/recording");
    timer_period_ms_ = this->declare_parameter<int>("timer_period_ms", 33);
    rotate_180_ = this->declare_parameter<bool>("rotate_180", false);
    show_window_ = this->declare_parameter<bool>("show_window", true);
    int acc_us = this->declare_parameter<int>("accumulation_time_us", 10000);

    // --- Core Logic Initialization ---
    core_ = std::make_unique<EventDriverCore>();
    core_->initialize_cameras(left_sn, right_sn, bias_file, acc_us, save_directory_);

    // --- ROS Subscriptions & Timers ---
    // Poll frames for visualization
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(timer_period_ms_),
        std::bind(&EventCameraNode::on_timer, this));

    // Listen for recording triggers from the master node
    status_sub_ = this->create_subscription<std_msgs::msg::String>(
        "recording_status", 10,
        std::bind(&EventCameraNode::on_status_msg, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Stereo Event Camera Node Initialized Successfully.");
  }

private:
  /**
   * @brief Timer callback for OpenCV visualization.
   */
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
   * @brief Subscriber callback to handle master recording signals.
   */
  void on_status_msg(const std_msgs::msg::String::SharedPtr msg) {
    if (msg->data == "start") {
      std::string path = core_->start_all_recording();
      RCLCPP_INFO(this->get_logger(), "Event Session Started: %s", path.c_str());
    } else if (msg->data == "stop") {
      core_->stop_all_recording();
      RCLCPP_INFO(this->get_logger(), "Event Session Stopped.");
    } else if (msg->data == "switch") {
      std::string path = core_->switch_all_directories();
      RCLCPP_INFO(this->get_logger(), "Event Session Switched: %s", path.c_str());
    }
  }

  std::unique_ptr<EventDriverCore> core_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr status_sub_;
  
  std::string save_directory_;
  int timer_period_ms_;
  bool rotate_180_;
  bool show_window_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<EventCameraNode>());
  rclcpp::shutdown();
  return 0;
}