#include "metro_application/interlock_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <memory>
#include <sstream>
#include <string>
#include <utility>

namespace metro
{
namespace application
{

namespace
{

constexpr const char * kFaultNotInWindow = "TRAIN_NOT_IN_STOP_WINDOW";
constexpr const char * kFaultPsdNotLocked = "PSD_NOT_LOCKED";
constexpr const char * kFaultOpenTimeout = "PSD_OPEN_TIMEOUT";
constexpr const char * kFaultEmergency = "EMERGENCY_BRAKE_PSD_OPEN";

std::string UnitServiceName(int _one_based_index)
{
  std::ostringstream stream;
  stream << "/metro/psd/unit_" << std::setw(2) << std::setfill('0')
         << _one_based_index << "/command";
  return stream.str();
}

}  // namespace

InterlockNode::InterlockNode()
: rclcpp::Node("interlock_node")
{
  DeclareParameters();

  const double control_rate_hz = std::max(1.0, ReadDouble("control_rate_hz"));
  const int unit_count = std::max(1, static_cast<int>(get_parameter(
    "psd_unit_count").as_int()));

  speed_config_.x_stop_m = ReadDouble("x_stop_m");
  speed_config_.cruise_speed_mps = ReadDouble("cruise_speed_mps");
  speed_config_.brake_decel_mps2 = ReadDouble("brake_decel_mps2");
  speed_config_.approach_margin_m = ReadDouble("approach_margin_m");
  speed_config_.max_speed_mps = ReadDouble("max_speed_mps");
  speed_controller_ = std::make_unique<TrainSpeedController>(speed_config_);

  train_command_publisher_ = create_publisher<geometry_msgs::msg::Twist>(
    "/metro/train/command", rclcpp::QoS(10).reliable());
  fault_publisher_ = create_publisher<std_msgs::msg::String>(
    "/metro/interlock/fault", rclcpp::QoS(10).reliable());

  odom_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
    "/metro/train/odom", rclcpp::SensorDataQoS(),
    std::bind(&InterlockNode::OnOdom, this, std::placeholders::_1));
  psd_state_subscription_ = create_subscription<std_msgs::msg::Bool>(
    "/metro/psd/state", rclcpp::QoS(10).reliable(),
    std::bind(&InterlockNode::OnPsdState, this, std::placeholders::_1));

  enable_service_ = create_service<std_srvs::srv::SetBool>(
    "/metro/interlock/enable",
    [this](
      const std::shared_ptr<std_srvs::srv::SetBool::Request> _request,
      std::shared_ptr<std_srvs::srv::SetBool::Response> _response)
    {
      this->OnEnable(_request, _response);
    });
  psd_command_service_ = create_service<std_srvs::srv::SetBool>(
    "/metro/psd/command",
    [this](
      const std::shared_ptr<std_srvs::srv::SetBool::Request> _request,
      std::shared_ptr<std_srvs::srv::SetBool::Response> _response)
    {
      this->OnPsdCommand(_request, _response);
    });

  fault_manager_ = std::make_unique<FaultManager>(
    get_clock(), fault_publisher_,
    ReadDouble("fault_hold_s"), ReadDouble("fault_repeat_suppression_s"));

  unit_clients_.reserve(static_cast<std::size_t>(unit_count));
  last_dispatched_.assign(static_cast<std::size_t>(unit_count), -1);
  for (int index = 1; index <= unit_count; ++index) {
    unit_clients_.push_back(
      create_client<std_srvs::srv::SetBool>(UnitServiceName(index)));
  }

  const auto period = std::chrono::duration<double>(1.0 / control_rate_hz);
  control_timer_ = create_wall_timer(period, std::bind(&InterlockNode::ControlLoop, this));

  LogParameters();
  RCLCPP_INFO(
    get_logger(),
    "Interlock ready: inputs /metro/train/odom (best_effort), "
    "/metro/psd/state, /metro/interlock/enable; outputs /metro/train/command, "
    "/metro/psd/command and /metro/interlock/fault. Units=%d",
    unit_count);
}

void InterlockNode::DeclareParameters()
{
  if (!has_parameter("use_sim_time")) {
    declare_parameter<bool>("use_sim_time", false);
  }
  declare_parameter<double>("stop_window_m", 0.5);
  declare_parameter<double>("psd_open_timeout_s", 3.0);
  declare_parameter<double>("emergency_speed_threshold_mps", 0.1);
  declare_parameter<double>("fault_hold_s", 1.0);
  declare_parameter<double>("fault_repeat_suppression_s", 5.0);
  declare_parameter<double>("control_rate_hz", 10.0);
  declare_parameter<int>("psd_unit_count", 4);
  declare_parameter<bool>("auto_run", true);
  declare_parameter<double>("manual_speed_mps", 0.0);
  declare_parameter<double>("x_stop_m", 0.0);
  declare_parameter<double>("cruise_speed_mps", 8.0);
  declare_parameter<double>("brake_decel_mps2", 1.0);
  declare_parameter<double>("approach_margin_m", 0.5);
  declare_parameter<double>("max_speed_mps", 20.0);
}

void InterlockNode::LogParameters() const
{
  RCLCPP_INFO(
    get_logger(),
    "parameters: stop_window_m=%.3f psd_open_timeout_s=%.3f "
    "emergency_speed_threshold_mps=%.3f fault_hold_s=%.3f "
    "fault_repeat_suppression_s=%.3f control_rate_hz=%.3f unit_count=%ld "
    "auto_run=%s manual_speed_mps=%.3f x_stop_m=%.3f cruise_speed_mps=%.3f "
    "brake_decel_mps2=%.3f approach_margin_m=%.3f max_speed_mps=%.3f",
    ReadDouble("stop_window_m"), ReadDouble("psd_open_timeout_s"),
    ReadDouble("emergency_speed_threshold_mps"), ReadDouble("fault_hold_s"),
    ReadDouble("fault_repeat_suppression_s"), ReadDouble("control_rate_hz"),
    static_cast<long>(get_parameter("psd_unit_count").as_int()),
    ReadBool("auto_run") ? "true" : "false", ReadDouble("manual_speed_mps"),
    ReadDouble("x_stop_m"), ReadDouble("cruise_speed_mps"),
    ReadDouble("brake_decel_mps2"), ReadDouble("approach_margin_m"),
    ReadDouble("max_speed_mps"));
}

bool InterlockNode::ReadBool(const std::string &_name) const
{
  return get_parameter(_name).as_bool();
}

double InterlockNode::ReadDouble(const std::string &_name) const
{
  return get_parameter(_name).as_double();
}

bool InterlockNode::InStopWindow() const
{
  if (!odom_received_) {
    return false;
  }
  return std::abs(train_x_m_ - ReadDouble("x_stop_m")) <= ReadDouble("stop_window_m");
}

void InterlockNode::OnOdom(const nav_msgs::msg::Odometry::SharedPtr _message)
{
  train_x_m_ = _message->pose.pose.position.x;
  train_v_mps_ = _message->twist.twist.linear.x;
  odom_received_ = true;
}

void InterlockNode::OnPsdState(const std_msgs::msg::Bool::SharedPtr _message)
{
  const bool was_locked = psd_locked_;
  psd_locked_ = _message->data;
  psd_received_ = true;
  if (!psd_locked_) {
    // R4 is satisfied as soon as the doors report "not fully closed/locked".
    open_request_time_.reset();
  }
  if (was_locked != psd_locked_) {
    RCLCPP_INFO(
      get_logger(), "/metro/psd/state -> %s",
      psd_locked_ ? "true (locked)" : "false (open/not locked)");
  }
}

void InterlockNode::OnEnable(
  const std::shared_ptr<std_srvs::srv::SetBool::Request> _request,
  std::shared_ptr<std_srvs::srv::SetBool::Response> _response)
{
  enabled_ = _request->data;
  if (!enabled_) {
    // R1: PSD command stays closed, train command passthrough, no fault.
    open_requested_ = false;
    open_request_time_.reset();
    RequestUnitCommand(false);
    fault_manager_->Clear();
    last_fault_code_.clear();
    RCLCPP_WARN(get_logger(), "R1: interlock disabled -> doors closing, faults cleared");
  } else {
    RCLCPP_INFO(get_logger(), "Interlock enabled");
  }

  _response->success = true;
  _response->message = enabled_ ? "interlock enabled" : "interlock disabled (R1: doors closed)";
}

void InterlockNode::OnPsdCommand(
  const std::shared_ptr<std_srvs::srv::SetBool::Request> _request,
  std::shared_ptr<std_srvs::srv::SetBool::Response> _response)
{
  if (_request->data) {
    if (!enabled_) {
      // R1: never open while disabled; keep the door command closed.
      RequestUnitCommand(false);
      _response->success = false;
      _response->message = "R1: interlock disabled, open refused";
      RCLCPP_WARN(get_logger(), "R1: open request refused while disabled");
      return;
    }

    if (!InStopWindow()) {
      // R2: reject opening outside the +/-0.5 m stop window.
      RaiseFault(kFaultNotInWindow);
      _response->success = false;
      _response->message = "R2: TRAIN_NOT_IN_STOP_WINDOW";
      RCLCPP_ERROR(
        get_logger(),
        "R2: open refused, x=%.3f target=%.3f window=%.3f",
        train_x_m_, ReadDouble("x_stop_m"), ReadDouble("stop_window_m"));
      return;
    }

    open_requested_ = true;
    open_request_time_ = get_clock()->now();
    RequestUnitCommand(true);
    _response->success = true;
    _response->message = "opening requested; watch /metro/psd/state";
    RCLCPP_INFO(get_logger(), "Open accepted at x=%.3f (R4 timer started)", train_x_m_);
    return;
  }

  open_requested_ = false;
  open_request_time_.reset();
  RequestUnitCommand(false);
  _response->success = true;
  _response->message = "closing requested";
  RCLCPP_INFO(get_logger(), "Close requested");
}

void InterlockNode::RequestUnitCommand(bool _open)
{
  pending_unit_command_ = _open;
  last_dispatched_.assign(unit_clients_.size(), -1);
}

void InterlockNode::DispatchPendingUnitCommand()
{
  if (!pending_unit_command_) {
    return;
  }

  const bool open = *pending_unit_command_;
  bool all_ready = true;
  for (std::size_t index = 0; index < unit_clients_.size(); ++index) {
    const auto &client = unit_clients_[index];
    if (!client->service_is_ready()) {
      all_ready = false;
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "unit %02zu command service not ready", index + 1);
      continue;
    }
    if (last_dispatched_[index] == (open ? 1 : 0)) {
      continue;
    }

    auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
    request->data = open;
    client->async_send_request(
      request,
      [this, index](rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture _future)
      {
        const auto response = _future.get();
        if (!response->success) {
          RCLCPP_ERROR(
            get_logger(), "unit %02zu rejected PSD command: %s",
            index + 1, response->message.c_str());
        }
      });
    last_dispatched_[index] = open ? 1 : 0;
  }

  if (all_ready) {
    pending_unit_command_.reset();
  }
}

void InterlockNode::RaiseFault(const std::string &_code)
{
  if (_code.empty()) {
    return;
  }
  if (fault_manager_->Raise(_code) && _code != last_fault_code_) {
    RCLCPP_ERROR(get_logger(), "fault: %s", _code.c_str());
    last_fault_code_ = _code;
  }
}

void InterlockNode::ControlLoop()
{
  const rclcpp::Time now = get_clock()->now();
  const double desired = odom_received_ ? speed_controller_->Compute(
    train_x_m_, train_v_mps_, ReadBool("auto_run"), ReadDouble("manual_speed_mps")) : 0.0;

  double safe_speed = desired;
  std::string fault_code;

  if (enabled_) {
    // R3/R5: a door that is not fully closed+locked always cuts traction.
    const bool psd_not_locked = psd_received_ && !psd_locked_;
    if (psd_not_locked) {
      safe_speed = 0.0;
      const double threshold = ReadDouble("emergency_speed_threshold_mps");
      if (std::abs(train_v_mps_) > threshold) {
        // R5 is the more specific emergency hazard; both R3 and R5 keep
        // traction at zero, but the published code is the emergency code.
        fault_code = kFaultEmergency;
      } else {
        fault_code = kFaultPsdNotLocked;
      }
    }

    // R4: while the command is open and the doors still claim locked, the
    // controller is not responding within psd_open_timeout_s.
    if (open_request_time_ && psd_locked_) {
      const rclcpp::Duration timeout = rclcpp::Duration::from_seconds(
        ReadDouble("psd_open_timeout_s"));
      if ((now - *open_request_time_) > timeout) {
        safe_speed = 0.0;
        fault_code = kFaultOpenTimeout;
      }
    }
  }

  geometry_msgs::msg::Twist command;
  // R1: disabled interlock passes the desired command through unchanged.
  command.linear.x = enabled_ ? safe_speed : desired;
  command.angular.x = 0.0;
  command.angular.y = 0.0;
  command.angular.z = 0.0;
  train_command_publisher_->publish(command);

  if (!fault_code.empty()) {
    RaiseFault(fault_code);
  }
  fault_manager_->Tick();
  DispatchPendingUnitCommand();
}

}  // namespace application
}  // namespace metro

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<metro::application::InterlockNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
