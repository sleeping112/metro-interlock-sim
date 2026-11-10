// PlatformMonitor: RViz MarkerArray visualisation for the enriched scene.
//
// Inputs
//   /metro/train/odom          nav_msgs/Odometry
//   /metro/psd/state           std_msgs/Bool
//   /metro/platform/intrusion  std_msgs/Bool
//
// Output
//   /metro/monitor/markers     visualization_msgs/MarkerArray, 10 Hz
//
// This node is a monitoring layer only; it never publishes a control command.
#include <algorithm>
#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/bool.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

namespace metro
{
namespace application
{

class PlatformMonitor : public rclcpp::Node
{
public:
  PlatformMonitor()
  : rclcpp::Node("platform_monitor")
  {
    if (!has_parameter("use_sim_time")) {
      declare_parameter<bool>("use_sim_time", false);
    }
    declare_parameter<int>("psd_unit_count", 12);
    declare_parameter<std::string>("station_name", "METRO CENTRAL");

    unit_count_ = std::max(1, static_cast<int>(get_parameter(
      "psd_unit_count").as_int()));
    station_name_ = get_parameter("station_name").as_string();

    odom_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
      "/metro/train/odom", rclcpp::SensorDataQoS(),
      [this](const nav_msgs::msg::Odometry::SharedPtr _message)
      {
        x_m_ = _message->pose.pose.position.x;
        v_mps_ = _message->twist.twist.linear.x;
      });
    psd_subscription_ = create_subscription<std_msgs::msg::Bool>(
      "/metro/psd/state", rclcpp::QoS(10).reliable(),
      [this](const std_msgs::msg::Bool::SharedPtr _message)
      {
        psd_locked_ = _message->data;
      });
    intrusion_subscription_ = create_subscription<std_msgs::msg::Bool>(
      "/metro/platform/intrusion", rclcpp::QoS(10).reliable(),
      [this](const std_msgs::msg::Bool::SharedPtr _message)
      {
        intrusion_ = _message->data;
      });

    marker_publisher_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      "/metro/monitor/markers", rclcpp::QoS(10).reliable());
    timer_ = create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&PlatformMonitor::PublishMarkers, this));

    RCLCPP_INFO(
      get_logger(), "monitor ready: units=%d station=%s",
      unit_count_, station_name_.c_str());
  }

private:
  static void SetColor(
    visualization_msgs::msg::Marker &_marker,
    float _r, float _g, float _b, float _a)
  {
    _marker.color.r = _r;
    _marker.color.g = _g;
    _marker.color.b = _b;
    _marker.color.a = _a;
  }

  void PublishMarkers()
  {
    visualization_msgs::msg::MarkerArray array;
    const auto stamp = get_clock()->now();

    visualization_msgs::msg::Marker train;
    train.header.frame_id = "station";
    train.header.stamp = stamp;
    train.ns = "train";
    train.id = 0;
    train.type = visualization_msgs::msg::Marker::CUBE;
    train.action = visualization_msgs::msg::Marker::ADD;
    train.pose.position.x = x_m_;
    train.pose.position.y = 8.65;
    train.pose.position.z = 1.5;
    train.pose.orientation.w = 1.0;
    train.scale.x = 120.0;
    train.scale.y = 3.0;
    train.scale.z = 3.2;
    SetColor(train, 0.20F, 0.55F, 0.95F, 0.35F);
    array.markers.push_back(train);

    for (int index = 1; index <= unit_count_; ++index) {
      visualization_msgs::msg::Marker door;
      door.header.frame_id = "station";
      door.header.stamp = stamp;
      door.ns = "psd";
      door.id = index;
      door.type = visualization_msgs::msg::Marker::CUBE;
      door.action = visualization_msgs::msg::Marker::ADD;
      door.pose.position.x = -55.0 + 10.0 * static_cast<double>(index - 1);
      door.pose.position.y = 7.05;
      door.pose.position.z = 1.3;
      door.pose.orientation.w = 1.0;
      door.scale.x = 0.30;
      door.scale.y = 0.10;
      door.scale.z = 2.2;
      if (psd_locked_) {
        SetColor(door, 0.15F, 0.90F, 0.30F, 0.85F);
      } else {
        SetColor(door, 0.95F, 0.20F, 0.15F, 0.85F);
      }
      array.markers.push_back(door);
    }

    visualization_msgs::msg::Marker status;
    status.header.frame_id = "station";
    status.header.stamp = stamp;
    status.ns = "status";
    status.id = 100;
    status.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    status.action = visualization_msgs::msg::Marker::ADD;
    status.pose.position.x = 0.0;
    status.pose.position.y = 2.0;
    status.pose.position.z = 3.8;
    status.pose.orientation.w = 1.0;
    status.scale.z = 0.55;
    status.text = intrusion_ ? "PLATFORM INTRUSION" : "PLATFORM CLEAR";
    SetColor(
      status, intrusion_ ? 0.95F : 0.20F, intrusion_ ? 0.15F : 0.90F,
      intrusion_ ? 0.10F : 0.30F, 0.95F);
    array.markers.push_back(status);

    visualization_msgs::msg::Marker title;
    title.header.frame_id = "station";
    title.header.stamp = stamp;
    title.ns = "status";
    title.id = 101;
    title.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    title.action = visualization_msgs::msg::Marker::ADD;
    title.pose.position.x = 0.0;
    title.pose.position.y = -1.5;
    title.pose.position.z = 4.4;
    title.pose.orientation.w = 1.0;
    title.scale.z = 0.6;
    title.text = station_name_;
    SetColor(title, 0.80F, 0.86F, 0.95F, 0.95F);
    array.markers.push_back(title);

    marker_publisher_->publish(array);
  }

  int unit_count_{12};
  std::string station_name_{"METRO CENTRAL"};
  double x_m_{0.0};
  double v_mps_{0.0};
  bool psd_locked_{true};
  bool intrusion_{false};

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr psd_subscription_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr intrusion_subscription_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace application
}  // namespace metro

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<metro::application::PlatformMonitor>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
