// Stand-alone train speed controller node (bench / manual mode).
//
// Inputs : /metro/train/odom (SensorDataQoS best_effort)
// Output : /metro/train/command (geometry_msgs/Twist) only if publish_command=true
//
// In the integrated demo the interlock node is the single writer of
// /metro/train/command and embeds the same TrainSpeedController library.  This
// executable defaults to publish_command=false to prevent two publishers from
// racing on the safety command topic.
#ifndef METRO_APPLICATION__TRAIN_SPEED_CONTROLLER_NODE_HPP_
#define METRO_APPLICATION__TRAIN_SPEED_CONTROLLER_NODE_HPP_

#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include "metro_application/train_speed_controller.hpp"

namespace metro
{
namespace application
{

class TrainSpeedControllerNode : public rclcpp::Node
{
public:
  TrainSpeedControllerNode();
  ~TrainSpeedControllerNode() override = default;

private:
  void OnOdom(const nav_msgs::msg::Odometry::SharedPtr _message);
  void ControlLoop();

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr command_publisher_;
  rclcpp::TimerBase::SharedPtr control_timer_;

  std::unique_ptr<TrainSpeedController> controller_;
  TrainSpeedConfig config_;

  bool publish_command_{false};
  bool odom_received_{false};
  double x_m_{0.0};
  double v_mps_{0.0};
};

}  // namespace application
}  // namespace metro

#endif  // METRO_APPLICATION__TRAIN_SPEED_CONTROLLER_NODE_HPP_
