#include "metro_application/train_speed_controller_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <string>

namespace metro
{
namespace application
{

TrainSpeedControllerNode::TrainSpeedControllerNode()
: rclcpp::Node("train_speed_controller_node")
{
  if (!has_parameter("use_sim_time")) {
    declare_parameter<bool>("use_sim_time", false);
  }
  declare_parameter<bool>("publish_command", false);
  declare_parameter<double>("control_rate_hz", 10.0);
  declare_parameter<double>("x_stop_m", 0.0);
  declare_parameter<double>("cruise_speed_mps", 8.0);
  declare_parameter<double>("brake_decel_mps2", 1.0);
  declare_parameter<double>("approach_margin_m", 0.5);
  declare_parameter<double>("max_speed_mps", 20.0);
  declare_parameter<bool>("auto_run", true);
  declare_parameter<double>("manual_speed_mps", 0.0);

  publish_command_ = get_parameter("publish_command").as_bool();
  const double control_rate_hz = std::max(
    1.0, get_parameter("control_rate_hz").as_double());

  config_.x_stop_m = get_parameter("x_stop_m").as_double();
  config_.cruise_speed_mps = get_parameter("cruise_speed_mps").as_double();
  config_.brake_decel_mps2 = get_parameter("brake_decel_mps2").as_double();
  config_.approach_margin_m = get_parameter("approach_margin_m").as_double();
  config_.max_speed_mps = get_parameter("max_speed_mps").as_double();
  controller_ = std::make_unique<TrainSpeedController>(config_);

  odom_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
    "/metro/train/odom", rclcpp::SensorDataQoS(),
    std::bind(&TrainSpeedControllerNode::OnOdom, this, std::placeholders::_1));
  command_publisher_ = create_publisher<geometry_msgs::msg::Twist>(
    "/metro/train/command", rclcpp::QoS(10).reliable());
  control_timer_ = create_wall_timer(
    std::chrono::duration<double>(1.0 / control_rate_hz),
    std::bind(&TrainSpeedControllerNode::ControlLoop, this));

  RCLCPP_INFO(
    get_logger(),
    "parameters: publish_command=%s x_stop_m=%.3f cruise_speed_mps=%.3f "
    "brake_decel_mps2=%.3f approach_margin_m=%.3f max_speed_mps=%.3f "
    "auto_run=%s manual_speed_mps=%.3f control_rate_hz=%.3f",
    publish_command_ ? "true" : "false", config_.x_stop_m,
    config_.cruise_speed_mps, config_.brake_decel_mps2,
    config_.approach_margin_m, config_.max_speed_mps,
    get_parameter("auto_run").as_bool() ? "true" : "false",
    get_parameter("manual_speed_mps").as_double(), control_rate_hz);

  if (!publish_command_) {
    RCLCPP_WARN(
      get_logger(),
      "publish_command=false: this bench node will NOT publish "
      "/metro/train/command. The interlock node is the single command writer "
      "in metro_demo.launch.py.");
  }
}

void TrainSpeedControllerNode::OnOdom(
  const nav_msgs::msg::Odometry::SharedPtr _message)
{
  x_m_ = _message->pose.pose.position.x;
  v_mps_ = _message->twist.twist.linear.x;
  odom_received_ = true;
}

void TrainSpeedControllerNode::ControlLoop()
{
  if (!publish_command_ || !odom_received_) {
    return;
  }

  const bool auto_run = get_parameter("auto_run").as_bool();
  const double manual_speed = get_parameter("manual_speed_mps").as_double();
  const double target = controller_->Compute(x_m_, v_mps_, auto_run, manual_speed);

  geometry_msgs::msg::Twist command;
  command.linear.x = target;
  command.angular.z = 0.0;
  command_publisher_->publish(command);
}

}  // namespace application
}  // namespace metro
