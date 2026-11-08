#include "metro_gazebo/door_controller.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>

#include <gz/plugin/Register.hh>
#include <gz/sim/Joint.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/components/Joint.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Name.hh>

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

std::string UnitId(std::size_t _one_based_index)
{
  std::ostringstream stream;
  stream << std::setw(2) << std::setfill('0') << _one_based_index;
  return stream.str();
}

}  // namespace

DoorController::~DoorController()
{
  services_.clear();
  state_pub_.reset();
  if (executor_ && node_) {
    executor_->remove_node(node_);
  }
  executor_.reset();
  node_.reset();
}

void DoorController::Configure(
  const gz::sim::Entity &_entity,
  const std::shared_ptr<const sdf::Element> &_sdf,
  gz::sim::EntityComponentManager &_ecm,
  gz::sim::EventManager &)
{
  Load(_entity, _sdf, _ecm);
}

void DoorController::Load(
  const gz::sim::Entity &_entity,
  const std::shared_ptr<const sdf::Element> &_sdf,
  gz::sim::EntityComponentManager &_ecm)
{
  world_entity_ = _entity;

  const int unit_count = std::max(1, SdfValue<int>(_sdf, "unit_count", 4));
  door_speed_mps_ = std::max(0.01, SdfValue<double>(_sdf, "door_speed_mps", 0.8));
  door_travel_m_ = std::max(0.01, SdfValue<double>(_sdf, "door_travel_m", 1.8));
  state_rate_hz_ = std::max(0.1, SdfValue<double>(_sdf, "state_rate_hz", 10.0));
  lock_delay_s_ = std::max(0.0, SdfValue<double>(_sdf, "lock_delay_s", 0.3));
  tolerance_m_ = std::max(1e-5, SdfValue<double>(
      _sdf, "open_position_tolerance_m", 0.01));

  const std::string model_prefix =
    SdfValue<std::string>(_sdf, "unit_model_prefix", "psd_door_");
  const std::string left_prefix =
    SdfValue<std::string>(_sdf, "left_joint_prefix", "psd_left_joint_");
  const std::string right_prefix =
    SdfValue<std::string>(_sdf, "right_joint_prefix", "psd_right_joint_");

  if (!rclcpp::ok()) {
    rclcpp::init(0, nullptr);
  }
  node_ = std::make_shared<rclcpp::Node>("metro_psd_door_controller");
  executor_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
  executor_->add_node(node_);
  state_pub_ = node_->create_publisher<std_msgs::msg::Bool>(
    "/metro/psd/state", rclcpp::QoS(10).reliable());

  units_.reserve(static_cast<std::size_t>(unit_count));
  for (int index = 1; index <= unit_count; ++index) {
    Unit unit;
    unit.id = UnitId(static_cast<std::size_t>(index));

    const std::string model_name = model_prefix + unit.id;
    unit.model = _ecm.EntityByComponents(
      gz::sim::components::Model(), gz::sim::components::Name(model_name));
    if (unit.model == gz::sim::kNullEntity) {
      std::cerr << "[DoorController] model not found: " << model_name << std::endl;
      units_.push_back(unit);
      continue;
    }

    gz::sim::Model model(unit.model);
    unit.left_joint = model.JointByName(_ecm, left_prefix + unit.id);
    unit.right_joint = model.JointByName(_ecm, right_prefix + unit.id);
    if (unit.left_joint == gz::sim::kNullEntity ||
        unit.right_joint == gz::sim::kNullEntity)
    {
      std::cerr << "[DoorController] joints not found for " << model_name
                << std::endl;
    } else {
      gz::sim::Joint(unit.left_joint).EnablePositionCheck(_ecm);
      gz::sim::Joint(unit.left_joint).EnableVelocityCheck(_ecm);
      gz::sim::Joint(unit.right_joint).EnablePositionCheck(_ecm);
      gz::sim::Joint(unit.right_joint).EnableVelocityCheck(_ecm);
      unit.position_check_enabled = true;
    }

    const std::size_t unit_index = units_.size();
    const std::string service_name = "/metro/psd/unit_" + unit.id + "/command";
    services_.push_back(node_->create_service<std_srvs::srv::SetBool>(
      service_name,
      [this, unit_index](
        const std::shared_ptr<std_srvs::srv::SetBool::Request> _request,
        std::shared_ptr<std_srvs::srv::SetBool::Response> _response)
      {
        this->OnUnitCommand(unit_index, _request, _response);
      }));

    std::cout << "[DoorController] unit " << unit.id << " model=" << model_name
              << " left_joint=" << unit.left_joint
              << " right_joint=" << unit.right_joint << std::endl;
    units_.push_back(unit);
  }

  std::cout << "[DoorController] units=" << units_.size()
            << " door_speed_mps=" << door_speed_mps_
            << " door_travel_m=" << door_travel_m_
            << " state_rate_hz=" << state_rate_hz_
            << " lock_delay_s=" << lock_delay_s_
            << " tolerance_m=" << tolerance_m_ << std::endl;
  std::cout << "[DoorController] output /metro/psd/state; services "
               "/metro/psd/unit_XX/command (one per unit)" << std::endl;
}

void DoorController::OnUnitCommand(
  std::size_t _index,
  const std::shared_ptr<std_srvs::srv::SetBool::Request> _request,
  std::shared_ptr<std_srvs::srv::SetBool::Response> _response)
{
  if (_index >= units_.size()) {
    _response->success = false;
    _response->message = "unknown PSD unit index";
    return;
  }

  {
    std::lock_guard<std::mutex> lock(unit_mutex_);
    units_[_index].target_open = _request->data;
  }
  _response->success = true;
  _response->message = _request->data ? "PSD unit opening" : "PSD unit closing";
  std::cout << "[DoorController] unit " << units_[_index].id
            << (_request->data ? " OPEN" : " CLOSE") << std::endl;
}

bool DoorController::GetJointPositions(
  const Unit &_unit,
  const gz::sim::EntityComponentManager &_ecm,
  double &_left,
  double &_right)
{
  if (_unit.left_joint == gz::sim::kNullEntity ||
      _unit.right_joint == gz::sim::kNullEntity)
  {
    return false;
  }

  const auto left_pos = gz::sim::Joint(_unit.left_joint).Position(_ecm);
  const auto right_pos = gz::sim::Joint(_unit.right_joint).Position(_ecm);
  if (!left_pos || !right_pos || left_pos->empty() || right_pos->empty()) {
    return false;
  }
  _left = left_pos->front();
  _right = right_pos->front();
  return true;
}

void DoorController::PreUpdate(
  const gz::sim::UpdateInfo &_info,
  gz::sim::EntityComponentManager &_ecm)
{
  const double dt = std::chrono::duration<double>(_info.dt).count();
  if (dt <= 0.0 || _info.paused) {
    return;
  }

  std::lock_guard<std::mutex> lock(unit_mutex_);
  for (auto &unit : units_) {
    if (unit.left_joint == gz::sim::kNullEntity ||
        unit.right_joint == gz::sim::kNullEntity)
    {
      continue;
    }

    double left = 0.0;
    double right = 0.0;
    if (!GetJointPositions(unit, _ecm, left, right)) {
      continue;
    }

    double left_velocity = 0.0;
    double right_velocity = 0.0;
    if (unit.target_open) {
      // Left leaf slides in -x, right leaf in +x.  The 1.8 m travel is also
      // encoded in the SDF joint limits.
      if (left > -door_travel_m_ + tolerance_m_) {
        left_velocity = -door_speed_mps_;
      }
      if (right < door_travel_m_ - tolerance_m_) {
        right_velocity = door_speed_mps_;
      }
    } else {
      if (left < -tolerance_m_) {
        left_velocity = door_speed_mps_;
      } else if (left > tolerance_m_) {
        left_velocity = -door_speed_mps_;
      }
      if (right > tolerance_m_) {
        right_velocity = -door_speed_mps_;
      } else if (right < -tolerance_m_) {
        right_velocity = door_speed_mps_;
      }
    }

    gz::sim::Joint(unit.left_joint).SetVelocity(_ecm, {left_velocity});
    gz::sim::Joint(unit.right_joint).SetVelocity(_ecm, {right_velocity});
  }

  // The velocity targets are refreshed every step; no integrator is required
  // here.  _info.simTime is the only clock used by this plugin.
}

void DoorController::PostUpdate(
  const gz::sim::UpdateInfo &_info,
  const gz::sim::EntityComponentManager &_ecm)
{
  if (executor_ && rclcpp::ok()) {
    executor_->spin_some();
  }

  const double now = std::chrono::duration<double>(_info.simTime).count();
  const double period = 1.0 / state_rate_hz_;
  if (state_initialized_ && (now - last_state_pub_time_) < period) {
    return;
  }
  state_initialized_ = true;
  last_state_pub_time_ = now;

  bool all_closed = !units_.empty();
  std::lock_guard<std::mutex> lock(unit_mutex_);
  for (const auto &unit : units_) {
    if (!unit.position_check_enabled) {
      all_closed = false;
      break;
    }
    double left = 0.0;
    double right = 0.0;
    if (!GetJointPositions(unit, _ecm, left, right)) {
      all_closed = false;
      break;
    }
    all_closed = all_closed &&
      std::abs(left) <= tolerance_m_ &&
      std::abs(right) <= tolerance_m_ &&
      !unit.target_open;
  }

  bool locked = false;
  if (all_closed) {
    if (closed_since_ < 0.0) {
      closed_since_ = now;
    }
    locked = (now - closed_since_) >= lock_delay_s_;
  } else {
    closed_since_ = -1.0;
  }

  // R3/R4/R5 consume this exact signal: false means "not fully closed+locked".
  if (locked != last_state_ || state_initialized_) {
    std_msgs::msg::Bool message;
    message.data = locked;
    state_pub_->publish(message);
    if (locked != last_state_) {
      std::cout << "[DoorController] /metro/psd/state -> "
                << (locked ? "true" : "false") << " at t=" << now << std::endl;
    }
    last_state_ = locked;
  }
}

}  // namespace gazebo
}  // namespace metro

GZ_ADD_PLUGIN(
  metro::gazebo::DoorController,
  gz::sim::System,
  metro::gazebo::DoorController::ISystemConfigure,
  metro::gazebo::DoorController::ISystemPreUpdate,
  metro::gazebo::DoorController::ISystemPostUpdate)

GZ_ADD_PLUGIN_ALIAS(
  metro::gazebo::DoorController,
  "metro::gazebo::DoorController")
