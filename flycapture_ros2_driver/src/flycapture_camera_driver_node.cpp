/**
 * @file flycapture_camera_driver_node.cpp
 * @brief Master Node that orchestrates sessions using a shared Session ID.
 */

#include "flycapture_camera_driver_core.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/set_bool.hpp"

using namespace flycapture_driver;

class FlyCaptureNode : public rclcpp::Node {
public:
  FlyCaptureNode() : Node("flycapture_driver_node") {
    // --- Parameters ---
    std::string r_sn = this->declare_parameter<std::string>("right_camera_serial", "");
    std::string l_sn = this->declare_parameter<std::string>("left_camera_serial", "");
    std::string save_dir = this->declare_parameter<std::string>("save_directory", "./images");
    std::string vm = this->declare_parameter<std::string>("video_mode", "VIDEOMODE_800x600YUV422");
    std::string hfr = this->declare_parameter<std::string>("hardware_frame_rate", "FRAMERATE_30");
    int timeout = this->declare_parameter<int>("timeout_ms", 1000);
    int timer_ms = this->declare_parameter<int>("timer_period_ms", 10);
    rotate_180_ = this->declare_parameter<bool>("rotate_180", false);

    core_ = std::make_unique<DriverCore>();
    core_->initialize_cameras(r_sn, l_sn, vm, hfr, timeout, save_dir);

    capture_timer_ = this->create_wall_timer(std::chrono::milliseconds(timer_ms), std::bind(&FlyCaptureNode::on_timer, this));
    status_pub_ = this->create_publisher<std_msgs::msg::String>("recording_status", 10);
    srv_ = this->create_service<std_srvs::srv::SetBool>("set_recording", std::bind(&FlyCaptureNode::on_service, this, std::placeholders::_1, std::placeholders::_2));
  }

private:
  void on_timer() {
    auto frames = core_->get_all_frames(rotate_180_);
    if (!frames.empty()) {
      cv::Mat combined; cv::hconcat(frames, combined);
      cv::imshow("RGB Stereo Preview", combined); cv::waitKey(1);
    }
  }

  void on_service(const std::shared_ptr<std_srvs::srv::SetBool::Request> req, std::shared_ptr<std_srvs::srv::SetBool::Response> res) {
    if (req->data) {
      // 1. Generate unique Session ID
      std::string session_id = core_->generate_session_name();
      // 2. Start local recording
      core_->start_all_recording(session_id);
      // 3. Broadcast "start:ID" to followers
      publish_status("start:" + session_id);
      start_switch_timer();
      res->message = "Recording started with ID: " + session_id;
    } else {
      core_->stop_all_recording();
      publish_status("stop");
      stop_switch_timer();
      res->message = "Recording stopped.";
    }
    res->success = true;
  }

  void start_switch_timer() {
    int interval = this->get_parameter("directory_switch_interval").as_int();
    switch_timer_ = this->create_wall_timer(std::chrono::seconds(interval), [this]() {
      core_->stop_all_recording();
      std::string new_id = core_->generate_session_name();
      core_->start_all_recording(new_id);
      publish_status("switch:" + new_id); // Broadcast "switch:ID"
    });
  }

  void stop_switch_timer() { if (switch_timer_) switch_timer_.reset(); }
  void publish_status(const std::string &s) { std_msgs::msg::String m; m.data = s; status_pub_->publish(m); }

  std::unique_ptr<DriverCore> core_;
  rclcpp::TimerBase::SharedPtr capture_timer_;
  rclcpp::TimerBase::SharedPtr switch_timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr srv_;
  bool rotate_180_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FlyCaptureNode>());
  rclcpp::shutdown(); return 0;
}