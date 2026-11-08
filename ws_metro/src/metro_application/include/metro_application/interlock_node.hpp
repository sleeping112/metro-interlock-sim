// InterlockNode: safety authority for the single-line metro station.
//
// Inputs
//   /metro/train/odom          nav_msgs/Odometry, SensorDataQoS best_effort
//   /metro/psd/state           std_msgs/Bool, reliable, true = all locked/closed
//   /metro/interlock/enable    std_srvs/SetBool
//   /metro/psd/command         std_srvs/SetBool (total door command, safety gated)
//
// Outputs
//   /metro/train/command       geometry_msgs/Twist, 10 Hz, linear.x only
//   /metro/interlock/fault     std_msgs/String, held 1 s, 5 s repeat suppression
//   /metro/psd/unit_XX/command fan-out to the DoorController plugin
//
// Rules are evaluated in this order: R1 (disabled), R2 (open outside window),
// R3 (doors not locked -> zero traction), R4 (open timeout), R5 (emergency brake
// if moving with doors open).  R3 and R5 both force zero; R5's more specific
// code wins when both conditions hold.
#ifndef METRO_APPLICATION__INTERLOCK_NODE_HPP_
#define METRO_APPLICATION__INTERLOCK_NODE_HPP_

#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include "metro_application/fault_manager.hpp"
#include "metro_application/train_speed_controller.hpp"

namespace metro
{
namespace application
{

class InterlockNode : public rclcpp::Node
{
public:
  InterlockNode();
  ~InterlockNode() override = default;

private:
  void OnOdom(const nav_msgs::msg::Odometry::SharedPtr _message);
  void OnPsdState(const std_msgs::msg::Bool::SharedPtr _message);
  void OnEnable(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> _request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> _response);
  void OnPsdCommand(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> _request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> _response);

  void ControlLoop();
  void RequestUnitCommand(bool _open);
  void DispatchPendingUnitCommand();
  void RaiseFault(const std::string &_code);
  bool InStopWindow() const;
  bool ReadBool(const std::string &_name) const;
  double ReadDouble(const std::string &_name) const;

  void DeclareParameters();
  void LogParameters() const;

  // Inputs.
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr psd_state_subscription_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr enable_service_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr psd_command_service_;

  // Outputs.
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr train_command_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr fault_publisher_;

  // Door-controller clients (one per unit; never a single aggregate hardware
  // service, matching the independent-unit contract).
  std::vector<rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr> unit_clients_;
  std::vector<int> last_dispatched_;
  std::optional<bool> pending_unit_command_;

  rclcpp::TimerBase::SharedPtr control_timer_;
  std::unique_ptr<FaultManager> fault_manager_;
  std::unique_ptr<TrainSpeedController> speed_controller_;
  TrainSpeedConfig speed_config_;

  bool enabled_{false};
  bool odom_received_{false};
  bool psd_received_{false};
  bool psd_locked_{true};
  double train_x_m_{0.0};
  double train_v_mps_{0.0};
  bool open_requested_{false};
  std::optional<rclcpp::Time> open_request_time_;
  std::string last_fault_code_;
};

}  // namespace application
}  // namespace metro

#endif  // METRO_APPLICATION__INTERLOCK_NODE_HPP_
