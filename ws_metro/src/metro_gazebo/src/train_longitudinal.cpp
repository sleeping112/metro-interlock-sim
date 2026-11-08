#include "metro_gazebo/train_longitudinal.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <string>

#include <gz/plugin/Register.hh>
#include <gz/sim/Joint.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/JointPosition.hh>
#include <gz/sim/components/JointVelocity.hh>

namespace metro
{
namespace gazebo
{

namespace
{

template<typename T>
T SdfValue(const std::shared_ptr<const sdf::Element> &_sdf,
           const std::string &_name, const T &_default)
{
  if (_sdf && _sdf->HasElement(_name)) {
    return _sdf->Get<T>(_name);
  }
  return _default;
}

void SetGzFrame(gz::msgs::Header &_header, const std::string &_frame)
{
  auto *frame = _header.add_data();
  frame->set_key("frame_id");
  frame->add_value(_frame);
}

}  // namespace

void TrainLongitudinal::Configure(
  const gz::sim::Entity &_entity,
  const std::shared_ptr<const sdf::Element> &_sdf,
  gz::sim::EntityComponentManager &_ecm,
  gz::sim::EventManager &)
{
  Load(_entity, _sdf, _ecm);
}

void TrainLongitudinal::Load(
  const gz::sim::Entity &_entity,
  const std::shared_ptr<const sdf::Element> &_sdf,
  gz::sim::EntityComponentManager &_ecm)
{
  model_entity_ = _entity;
  odom_topic_ = SdfValue<std::string>(_sdf, "odom_topic", "/metro/train/odom");
  command_topic_ = SdfValue<std::string>(
    _sdf, "command_topic", "/metro/train/command");
  odom_rate_hz_ = std::max(0.1, SdfValue<double>(_sdf, "odom_rate_hz", 50.0));
  max_speed_mps_ = std::max(0.1, SdfValue<double>(_sdf, "max_speed_mps", 20.0));
  max_accel_mps2_ = std::max(0.01, SdfValue<double>(_sdf, "max_accel_mps2", 1.0));
  max_decel_mps2_ = std::max(0.01, SdfValue<double>(_sdf, "max_decel_mps2", 2.0));
  rail_y_m_ = SdfValue<double>(_sdf, "rail_y_m", 6.65);

  initial_x_m_ = gz::sim::worldPose(model_entity_, _ecm).Pos().X();

  gz::sim::Model model(model_entity_);
  joint_entity_ = model.JointByName(_ecm, "train_longitudinal");
  if (joint_entity_ == gz::sim::kNullEntity) {
    std::cerr << "[TrainLongitudinal] joint train_longitudinal not found"
              << std::endl;
  } else {
    gz::sim::Joint(joint_entity_).EnablePositionCheck(_ecm);
    gz::sim::Joint(joint_entity_).EnableVelocityCheck(_ecm);
  }

  if (!transport_node_.Subscribe(
        command_topic_, &TrainLongitudinal::OnCommand, this))
  {
    std::cerr << "[TrainLongitudinal] failed to subscribe to "
              << command_topic_ << std::endl;
  }
  odom_publisher_ = transport_node_.Advertise<gz::msgs::Odometry>(odom_topic_);

  std::cout << "[TrainLongitudinal] model=" << model_entity_
            << " joint=" << joint_entity_
            << " initial_x_m=" << initial_x_m_
            << " rail_y_m=" << rail_y_m_
            << " max_speed_mps=" << max_speed_mps_
            << " max_accel_mps2=" << max_accel_mps2_
            << " max_decel_mps2=" << max_decel_mps2_
            << " odom_rate_hz=" << odom_rate_hz_
            << " command_topic=" << command_topic_
            << " odom_topic=" << odom_topic_ << std::endl;
  std::cout << "[TrainLongitudinal] applies only Twist.linear.x; angular is "
               "ignored. Effective /metro/train/command arbitration is done "
               "by interlock_node (R1/R3/R5)." << std::endl;
}

void TrainLongitudinal::OnCommand(const gz::msgs::Twist &_msg)
{
  // Only the longitudinal channel is part of the one-dimensional model.
  this->commanded_speed_mps_.store(_msg.linear().x());
}

void TrainLongitudinal::PreUpdate(
  const gz::sim::UpdateInfo &_info,
  gz::sim::EntityComponentManager &_ecm)
{
  if (joint_entity_ == gz::sim::kNullEntity || _info.paused) {
    return;
  }

  const auto velocity = gz::sim::Joint(joint_entity_).Velocity(_ecm);
  if (velocity && !velocity->empty()) {
    measured_velocity_mps_ = velocity->front();
  }

  double target = std::clamp(
    this->commanded_speed_mps_.load(), -max_speed_mps_, max_speed_mps_);
  const double dt = std::chrono::duration<double>(_info.dt).count();

  if (dt > 0.0) {
    const double max_rise = max_accel_mps2_ * dt;
    const double max_fall = max_decel_mps2_ * dt;
    if (target > measured_velocity_mps_ + max_rise) {
      target = measured_velocity_mps_ + max_rise;
    } else if (target < measured_velocity_mps_ - max_fall) {
      target = measured_velocity_mps_ - max_fall;
    }
  }

  if (std::abs(target) < 1e-3 && std::abs(commanded_speed_mps_.load()) < 1e-3) {
    target = 0.0;
  }

  gz::sim::Joint(joint_entity_).SetVelocity(_ecm, {target});
}

void TrainLongitudinal::PostUpdate(
  const gz::sim::UpdateInfo &_info,
  const gz::sim::EntityComponentManager &_ecm)
{
  if (joint_entity_ == gz::sim::kNullEntity) {
    return;
  }

  if (!initial_pose_initialized_) {
    // The spawn pose (-x in the launch) may not yet be reflected in the ECM
    // when Configure() is called; latch it on the first PostUpdate.
    initial_x_m_ = gz::sim::worldPose(model_entity_, _ecm).Pos().X();
    initial_pose_initialized_ = true;
    std::cout << "[TrainLongitudinal] latched initial_x_m=" << initial_x_m_
              << std::endl;
  }

  const double now = std::chrono::duration<double>(_info.simTime).count();
  const double period = 1.0 / odom_rate_hz_;
  if (last_odom_time_s_ >= 0.0 && (now - last_odom_time_s_) < period) {
    return;
  }
  last_odom_time_s_ = now;

  const auto position = gz::sim::Joint(joint_entity_).Position(_ecm);
  if (!position || position->empty()) {
    return;
  }

  gz::msgs::Odometry message;
  auto *stamp = message.mutable_header()->mutable_stamp();
  const auto stamp_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
    _info.simTime).count();
  stamp->set_sec(static_cast<int64_t>(stamp_ns / 1000000000LL));
  stamp->set_nsec(static_cast<int32_t>(stamp_ns % 1000000000LL));
  SetGzFrame(*message.mutable_header(), "station");
  auto *child = message.mutable_header()->add_data();
  child->set_key("child_frame_id");
  child->add_value("metro_train/train_body");

  auto *pose = message.mutable_pose();
  pose->mutable_position()->set_x(initial_x_m_ + position->front());
  pose->mutable_position()->set_y(rail_y_m_);
  pose->mutable_position()->set_z(0.0);
  pose->mutable_orientation()->set_x(0.0);
  pose->mutable_orientation()->set_y(0.0);
  pose->mutable_orientation()->set_z(0.0);
  pose->mutable_orientation()->set_w(1.0);

  const auto velocity = gz::sim::Joint(joint_entity_).Velocity(_ecm);
  const double measured = (velocity && !velocity->empty()) ?
    velocity->front() : measured_velocity_mps_;
  auto *twist = message.mutable_twist();
  twist->mutable_linear()->set_x(measured);

  odom_publisher_.Publish(message);
}

}  // namespace gazebo
}  // namespace metro

GZ_ADD_PLUGIN(
  metro::gazebo::TrainLongitudinal,
  gz::sim::System,
  metro::gazebo::TrainLongitudinal::ISystemConfigure,
  metro::gazebo::TrainLongitudinal::ISystemPreUpdate,
  metro::gazebo::TrainLongitudinal::ISystemPostUpdate)

GZ_ADD_PLUGIN_ALIAS(
  metro::gazebo::TrainLongitudinal,
  "metro::gazebo::TrainLongitudinal")
