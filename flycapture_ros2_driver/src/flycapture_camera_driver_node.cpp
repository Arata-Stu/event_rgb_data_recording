#include "flycapture_camera_driver_core.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/set_bool.hpp"

using namespace flycapture_driver;

class FlyCaptureNode : public rclcpp::Node {
public:
  FlyCaptureNode() : Node("flycapture_driver_node") {
    // --- Parameter Declaration ---
    this->declare_parameter<std::string>("right_camera_serial", "");
    this->declare_parameter<std::string>("left_camera_serial", "");
    this->declare_parameter<std::string>("save_directory", "./images");
    this->declare_parameter<std::string>("video_mode", "VIDEOMODE_800x600YUV422");
    this->declare_parameter<std::string>("hardware_frame_rate", "FRAMERATE_30");
    this->declare_parameter<int>("timeout_ms", 1000);
    this->declare_parameter<int>("timer_period_ms", 10); // Polling frequency
    this->declare_parameter<int>("switch_interval_sec", 60);
    this->declare_parameter<bool>("show_window", true);
    this->declare_parameter<bool>("rotate_180", false);

    // --- Core Initialization ---
    core_ = std::make_unique<DriverCore>();
    core_->initialize_cameras(
        this->get_parameter("right_camera_serial").as_string(),
        this->get_parameter("left_camera_serial").as_string(),
        this->get_parameter("video_mode").as_string(),
        this->get_parameter("hardware_frame_rate").as_string(),
        this->get_parameter("timeout_ms").as_int(),
        this->get_parameter("save_directory").as_string());

    // --- ROS Interfaces ---
    int timer_ms = this->get_parameter("timer_period_ms").as_int();
    capture_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(timer_ms),
        std::bind(&FlyCaptureNode::capture_callback, this));

    record_service_ = this->create_service<std_srvs::srv::SetBool>(
        "set_recording",
        std::bind(&FlyCaptureNode::handle_recording_service, this, std::placeholders::_1, std::placeholders::_2));

    status_pub_ = this->create_publisher<std_msgs::msg::String>("recording_status", 10);
  }

private:
  void capture_callback() {
    bool rotate = this->get_parameter("rotate_180").as_bool();
    auto frames = core_->get_all_frames(rotate);

    if (!frames.empty() && this->get_parameter("show_window").as_bool()) {
      cv::Mat combined;
      cv::hconcat(frames, combined);
      cv::imshow("FlyCapture Multi-View", combined);
      cv::waitKey(1);
    }
  }

  void handle_recording_service(const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
                                std::shared_ptr<std_srvs::srv::SetBool::Response> res) {
    if (req->data) {
      std::string path = core_->start_all_recording();
      start_switch_timer();
      res->success = true;
      res->message = "Recording started: " + path;
      publish_status("start");
    } else {
      core_->stop_all_recording();
      stop_switch_timer();
      res->success = true;
      res->message = "Recording stopped";
      publish_status("stop");
    }
  }

  void start_switch_timer() {
    int interval = this->get_parameter("switch_interval_sec").as_int();
    switch_timer_ = this->create_wall_timer(
        std::chrono::seconds(interval),
        [this]() {
          core_->switch_all_directories();
          publish_status("switch");
        });
  }

  void stop_switch_timer() {
    if (switch_timer_)
      switch_timer_.reset();
  }

  void publish_status(const std::string &status) {
    auto msg = std_msgs::msg::String();
    msg.data = status;
    status_pub_->publish(msg);
  }

  std::unique_ptr<DriverCore> core_;
  rclcpp::TimerBase::SharedPtr capture_timer_;
  rclcpp::TimerBase::SharedPtr switch_timer_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr record_service_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<FlyCaptureNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}