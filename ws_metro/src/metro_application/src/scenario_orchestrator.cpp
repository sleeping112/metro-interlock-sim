// ScenarioOrchestrator: one automated arrival - dwell - departure cycle.
//
// Inputs
//   /metro/train/odom          nav_msgs/Odometry (sensor QoS)
//   /metro/psd/state           std_msgs/Bool (true=locked)
//   /metro/interlock/enable    std_srvs/SetBool client
//   /metro/psd/command         std_srvs/SetBool client
//   /interlock_node/set_parameters (rcl_interfaces, standard ROS parameter service)
//
// Output actions
//   enable interlock -> wait for automatic stop -> open PSD -> dwell ->
//   close PSD -> wait for lock -> request departure via interlock parameters.
//
// No custom message or scenario-specific public topic is introduced.  The
// interlock remains the sole publisher of /metro/train/command.
#include <algorithm>
#include <chrono>
#include <cmath>
#include <future>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/parameter.hpp>
#include <rcl_interfaces/msg/parameter_type.hpp>
#include <rcl_interfaces/msg/parameter_value.hpp>
#include <rcl_interfaces/srv/set_parameters.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <nav_msgs/msg/odometry.hpp>

namespace metro
{
namespace application
{

class ScenarioOrchestrator : public rclcpp::Node
{
public:
  ScenarioOrchestrator()
  : rclcpp::Node("scenario_orchestrator")
  {
    if (!has_parameter("use_sim_time")) {
      declare_parameter<bool>("use_sim_time", false);
    }
    declare_parameter<double>("dwell_time_s", 20.0);
    declare_parameter<double>("departure_speed_mps", 6.0);
    declare_parameter<double>("departure_distance_m", 65.0);
    declare_parameter<double>("open_timeout_s", 5.0);
    declare_parameter<bool>("auto_start", true);

    dwell_time_s_ = get_parameter("dwell_time_s").as_double();
    departure_speed_mps_ = get_parameter("departure_speed_mps").as_double();
    departure_distance_m_ = get_parameter("departure_distance_m").as_double();
    open_timeout_s_ = get_parameter("open_timeout_s").as_double();
    auto_start_ = get_parameter("auto_start").as_bool();

    odom_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
      "/metro/train/odom", rclcpp::SensorDataQoS(),
      [this](const nav_msgs::msg::Odometry::SharedPtr _message)
      {
        x_m_ = _message->pose.pose.position.x;
        v_mps_ = _message->twist.twist.linear.x;
        odom_received_ = true;
      });
    psd_subscription_ = create_subscription<std_msgs::msg::Bool>(
      "/metro/psd/state", rclcpp::QoS(10).reliable(),
      [this](const std_msgs::msg::Bool::SharedPtr _message)
      {
        psd_locked_ = _message->data;
        psd_received_ = true;
      });

    enable_client_ = create_client<std_srvs::srv::SetBool>(
      "/metro/interlock/enable");
    psd_client_ = create_client<std_srvs::srv::SetBool>("/metro/psd/command");
    parameter_client_ =
      create_client<rcl_interfaces::srv::SetParameters>(
      "/interlock_node/set_parameters");

    startup_time_ = get_clock()->now();
    timer_ = create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&ScenarioOrchestrator::Tick, this));

    RCLCPP_INFO(
      get_logger(),
      "scenario parameters: dwell=%.1fs departure=%.1fm/s distance=%.1fm "
      "open_timeout=%.1fs auto_start=%s",
      dwell_time_s_, departure_speed_mps_, departure_distance_m_,
      open_timeout_s_, auto_start_ ? "true" : "false");
  }

private:
  enum class State
  {
    WAIT_READY,
    ENABLING,
    WAIT_STOP,
    OPENING,
    WAIT_OPEN,
    DWELL,
    CLOSING,
    WAIT_LOCKED,
    DEPARTING,
    WAIT_DEPART,
    DONE,
    ERROR
  };

  using BoolFuture = rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture;
  using ParamFuture =
    rclcpp::Client<rcl_interfaces::srv::SetParameters>::SharedFuture;

  void Tick()
  {
    if (!auto_start_ && state_ == State::WAIT_READY) {
      return;
    }

    const rclcpp::Time now = get_clock()->now();
    switch (state_) {
      case State::WAIT_READY:
        if (odom_received_ && psd_received_ && psd_locked_ &&
            (now - startup_time_) > rclcpp::Duration::from_seconds(2.0) &&
            enable_client_->service_is_ready() &&
            psd_client_->service_is_ready() &&
            parameter_client_->service_is_ready())
        {
          SetInterlockParams(true, 0.0);
          auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
          request->data = true;
          enable_future_ = enable_client_->async_send_request(request).share();
          state_ = State::ENABLING;
          RCLCPP_INFO(get_logger(), "scenario: enabling interlock");
        }
        break;

      case State::ENABLING:
        if (PollBool(enable_future_, "interlock enable")) {
          state_ = State::WAIT_STOP;
          RCLCPP_INFO(get_logger(), "scenario: waiting for train to stop at x=0");
        }
        break;

      case State::WAIT_STOP:
        if (std::abs(x_m_) <= 0.5 && std::abs(v_mps_) < 0.05) {
          auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
          request->data = true;
          psd_future_ = psd_client_->async_send_request(request).share();
          open_request_time_ = now;
          state_ = State::OPENING;
          RCLCPP_INFO(
            get_logger(), "scenario: train stopped at x=%.3f, opening PSD",
            x_m_);
        }
        break;

      case State::OPENING:
        if (PollBool(psd_future_, "PSD open")) {
          state_ = State::WAIT_OPEN;
        }
        break;

      case State::WAIT_OPEN:
        if (psd_received_ && !psd_locked_) {
          dwell_start_ = now;
          state_ = State::DWELL;
          RCLCPP_INFO(get_logger(), "scenario: PSD open, dwell timer started");
        } else if ((now - open_request_time_) >
          rclcpp::Duration::from_seconds(open_timeout_s_))
        {
          RCLCPP_ERROR(get_logger(), "scenario: PSD open timeout");
          state_ = State::ERROR;
        }
        break;

      case State::DWELL:
        if ((now - dwell_start_) >
          rclcpp::Duration::from_seconds(dwell_time_s_))
        {
          auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
          request->data = false;
          psd_future_ = psd_client_->async_send_request(request).share();
          state_ = State::CLOSING;
          RCLCPP_INFO(get_logger(), "scenario: dwell finished, closing PSD");
        }
        break;

      case State::CLOSING:
        if (PollBool(psd_future_, "PSD close")) {
          state_ = State::WAIT_LOCKED;
        }
        break;

      case State::WAIT_LOCKED:
        if (psd_received_ && psd_locked_) {
          SetInterlockParams(false, departure_speed_mps_);
          state_ = State::DEPARTING;
          RCLCPP_INFO(
            get_logger(), "scenario: PSD locked, requesting departure at %.1f m/s",
            departure_speed_mps_);
        }
        break;

      case State::DEPARTING:
        if (PollParams("departure parameters")) {
          state_ = State::WAIT_DEPART;
        }
        break;

      case State::WAIT_DEPART:
        if (x_m_ > departure_distance_m_) {
          SetInterlockParams(false, 0.0);
          state_ = State::DONE;
          RCLCPP_INFO(
            get_logger(), "scenario: train departed to x=%.1f; cycle complete",
            x_m_);
        }
        break;

      case State::DONE:
      case State::ERROR:
        break;
    }
  }

  bool PollBool(std::optional<BoolFuture> &_future, const std::string &_what)
  {
    if (!_future) {
      return false;
    }
    if (_future->wait_for(std::chrono::seconds(0)) != std::future_status::ready) {
      return false;
    }
    const auto response = _future->get();
    _future.reset();
    if (!response->success) {
      RCLCPP_ERROR(
        get_logger(), "scenario: %s rejected: %s", _what.c_str(),
        response->message.c_str());
      state_ = State::ERROR;
      return false;
    }
    return true;
  }

  bool PollParams(const std::string &_what)
  {
    if (!param_future_) {
      return false;
    }
    if (param_future_->wait_for(std::chrono::seconds(0)) !=
      std::future_status::ready)
    {
      return false;
    }
    const auto response = param_future_->get();
    param_future_.reset();
    const bool ok = !response->results.empty() &&
      std::all_of(
      response->results.begin(), response->results.end(),
      [](const auto &_result) {return _result.successful;});
    if (!ok) {
      RCLCPP_ERROR(get_logger(), "scenario: %s rejected", _what.c_str());
      state_ = State::ERROR;
    }
    return ok;
  }

  void SetInterlockParams(bool _auto_run, double _manual_speed)
  {
    auto request =
      std::make_shared<rcl_interfaces::srv::SetParameters::Request>();

    rcl_interfaces::msg::Parameter auto_run;
    auto_run.name = "auto_run";
    auto_run.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_BOOL;
    auto_run.value.bool_value = _auto_run;
    request->parameters.push_back(auto_run);

    rcl_interfaces::msg::Parameter manual;
    manual.name = "manual_speed_mps";
    manual.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
    manual.value.double_value = _manual_speed;
    request->parameters.push_back(manual);

    param_future_ = parameter_client_->async_send_request(request).share();
  }

  State state_{State::WAIT_READY};
  bool odom_received_{false};
  bool psd_received_{false};
  bool psd_locked_{true};
  bool auto_start_{true};
  double x_m_{0.0};
  double v_mps_{0.0};
  double dwell_time_s_{20.0};
  double departure_speed_mps_{6.0};
  double departure_distance_m_{65.0};
  double open_timeout_s_{5.0};

  rclcpp::Time open_request_time_;
  rclcpp::Time dwell_start_;
  rclcpp::Time startup_time_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr psd_subscription_;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr enable_client_;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr psd_client_;
  rclcpp::Client<rcl_interfaces::srv::SetParameters>::SharedPtr parameter_client_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::optional<BoolFuture> enable_future_;
  std::optional<BoolFuture> psd_future_;
  std::optional<ParamFuture> param_future_;
};

}  // namespace application
}  // namespace metro

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<metro::application::ScenarioOrchestrator>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
