// PassengerFlow: deterministic low-poly passenger boarding animation.
//
// Inputs  : /metro/psd/state (std_msgs/Bool, true=locked; false starts flow)
// Outputs : none (moves the passenger_XX models in the Gazebo scene)
//
// The plugin starts one boarding wave on the falling edge of PSD state.  Each
// passenger walks from the platform to a different PSD unit, crosses the sill
// and disappears inside the train.  No ROS command topics are added.
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include <gz/plugin/Register.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/Pose.hh>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>

namespace metro
{
namespace gazebo
{

namespace
{

std::string TwoDigit(int _value)
{
  std::ostringstream stream;
  stream << std::setw(2) << std::setfill('0') << _value;
  return stream.str();
}

double Interpolate(double _a, double _b, double _u)
{
  const double clamped = std::clamp(_u, 0.0, 1.0);
  return _a + (_b - _a) * clamped;
}

gz::math::Pose3d WalkPose(
  const gz::math::Pose3d &_start, double _door_x, double _t)
{
  const double x0 = _start.Pos().X();
  const double y0 = _start.Pos().Y();
  double x = x0;
  double y = y0;
  double z = 0.30;

  if (_t < 2.5) {
    const double u = _t / 2.5;
    x = Interpolate(x0, _door_x - 0.65, u);
    y = Interpolate(y0, 5.55, u);
  } else if (_t < 4.5) {
    const double u = (_t - 2.5) / 2.0;
    x = Interpolate(_door_x - 0.65, _door_x, u);
    y = Interpolate(5.55, 6.55, u);
  } else if (_t < 5.6) {
    const double u = (_t - 4.5) / 1.1;
    x = _door_x;
    y = Interpolate(6.55, 7.25, u);
  } else {
    x = _door_x;
    y = 7.25;
    z = -20.0 - (_t - 5.6);
  }
  return gz::math::Pose3d(x, y, z, 0.0, 0.0, 0.0);
}

}  // namespace

class PassengerFlow
  : public gz::sim::System,
    public gz::sim::ISystemConfigure,
    public gz::sim::ISystemPreUpdate,
    public gz::sim::ISystemPostUpdate
{
public:
  void Configure(
    const gz::sim::Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    gz::sim::EntityComponentManager &_ecm,
    gz::sim::EventManager &) override
  {
    if (_sdf) {
      if (_sdf->HasElement("passenger_prefix")) {
        passenger_prefix_ = _sdf->Get<std::string>("passenger_prefix");
      }
      if (_sdf->HasElement("passenger_count")) {
        passenger_count_ = std::max(1, _sdf->Get<int>("passenger_count"));
      }
      if (_sdf->HasElement("door_start_x")) {
        door_start_x_ = _sdf->Get<double>("door_start_x");
      }
      if (_sdf->HasElement("door_spacing_m")) {
        door_spacing_m_ = _sdf->Get<double>("door_spacing_m");
      }
      if (_sdf->HasElement("door_y_m")) {
        door_y_m_ = _sdf->Get<double>("door_y_m");
      }
      if (_sdf->HasElement("walk_speed_mps")) {
        walk_speed_mps_ = std::max(0.2, _sdf->Get<double>("walk_speed_mps"));
      }
      if (_sdf->HasElement("start_delay_s")) {
        start_delay_s_ = std::max(0.0, _sdf->Get<double>("start_delay_s"));
      }
      if (_sdf->HasElement("stagger_s")) {
        stagger_s_ = std::max(0.0, _sdf->Get<double>("stagger_s"));
      }
    }

    for (int index = 1; index <= passenger_count_; ++index) {
      const std::string name = passenger_prefix_ + TwoDigit(index);
      const auto entity = _ecm.EntityByComponents(
        gz::sim::components::Model(), gz::sim::components::Name(name));
      if (entity == gz::sim::kNullEntity) {
        continue;
      }
      Passenger passenger;
      passenger.entity = entity;
      passenger.start = gz::sim::worldPose(entity, _ecm);
      passenger.door_x = door_start_x_ +
        static_cast<double>((index - 1) % 12) * door_spacing_m_;
      passengers_.push_back(passenger);
    }

    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
    node_ = std::make_shared<rclcpp::Node>("metro_passenger_flow");
    executor_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(node_);
    subscription_ = node_->create_subscription<std_msgs::msg::Bool>(
      "/metro/psd/state", rclcpp::QoS(10).reliable(),
      [this](const std_msgs::msg::Bool::SharedPtr _message)
      {
        const bool now_open = !_message->data;
        const bool was_open = doors_open_.exchange(now_open);
        if (!was_open && now_open) {
          start_requested_.store(true);
        }
      });

    std::cout << "[PassengerFlow] passengers=" << passengers_.size()
              << " door_start_x=" << door_start_x_
              << " spacing=" << door_spacing_m_
              << " door_y=" << door_y_m_
              << " speed=" << walk_speed_mps_ << std::endl;
  }

  void PreUpdate(
    const gz::sim::UpdateInfo &_info,
    gz::sim::EntityComponentManager &_ecm) override
  {
    if (_info.paused || passengers_.empty()) {
      return;
    }

    const double now = std::chrono::duration<double>(_info.simTime).count();
    if (start_requested_.exchange(false) && !started_) {
      started_ = true;
      start_time_s_ = now + start_delay_s_;
      std::cout << "[PassengerFlow] boarding wave started at t="
                << start_time_s_ << std::endl;
    }
    if (!started_) {
      return;
    }

    for (std::size_t i = 0; i < passengers_.size(); ++i) {
      const double local_t = now - start_time_s_ - stagger_s_ * static_cast<double>(i);
      if (local_t < 0.0) {
        continue;
      }
      const gz::math::Pose3d pose = WalkPose(
        passengers_[i].start, passengers_[i].door_x, local_t);
      _ecm.SetComponentData<gz::sim::components::Pose>(
        passengers_[i].entity, pose);
    }
  }

  void PostUpdate(
    const gz::sim::UpdateInfo &,
    const gz::sim::EntityComponentManager &) override
  {
    if (executor_ && rclcpp::ok()) {
      executor_->spin_some();
    }
  }

private:
  struct Passenger
  {
    gz::sim::Entity entity{gz::sim::kNullEntity};
    gz::math::Pose3d start;
    double door_x{0.0};
  };

  std::string passenger_prefix_{"passenger_"};
  int passenger_count_{12};
  double door_start_x_{-55.0};
  double door_spacing_m_{10.0};
  double door_y_m_{6.90};
  double walk_speed_mps_{1.25};
  double start_delay_s_{0.8};
  double stagger_s_{0.35};
  std::vector<Passenger> passengers_;

  std::atomic<bool> doors_open_{true};
  std::atomic<bool> start_requested_{false};
  bool started_{false};
  double start_time_s_{-1.0};

  std::shared_ptr<rclcpp::Node> node_;
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr subscription_;
};

}  // namespace gazebo
}  // namespace metro

GZ_ADD_PLUGIN(
  metro::gazebo::PassengerFlow,
  gz::sim::System,
  metro::gazebo::PassengerFlow::ISystemConfigure,
  metro::gazebo::PassengerFlow::ISystemPreUpdate,
  metro::gazebo::PassengerFlow::ISystemPostUpdate)

GZ_ADD_PLUGIN_ALIAS(
  metro::gazebo::PassengerFlow,
  "metro::gazebo::PassengerFlow")
