// DoorController: Gazebo world system plugin for the platform screen doors.
//
// Inputs  : /metro/psd/unit_XX/command  (std_srvs/SetBool, true=open, false=close)
// Outputs : /metro/psd/state            (std_msgs/Bool, 10 Hz, true=all closed+locked)
// Rules   : publishes the R3/R4/R5 input signal; command services only move the
//           low-level leaves.  Safety decisions live in the interlock node.
//
// The four units are independent models (psd_door_01..04); each leaf is a
// prismatic body whose joint is driven with Joint::SetVelocity.  Services and
// the publisher are created in Load(), called from Configure(); Load() never
// blocks or sleeps.
#ifndef METRO_GAZEBO__DOOR_CONTROLLER_HPP_
#define METRO_GAZEBO__DOOR_CONTROLLER_HPP_

#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <gz/sim/System.hh>
#include <gz/sim/Entity.hh>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_srvs/srv/set_bool.hpp>

namespace metro
{
namespace gazebo
{

class DoorController
  : public gz::sim::System,
    public gz::sim::ISystemConfigure,
    public gz::sim::ISystemPreUpdate,
    public gz::sim::ISystemPostUpdate
{
public:
  DoorController() = default;
  ~DoorController() override;

  void Configure(
    const gz::sim::Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    gz::sim::EntityComponentManager &_ecm,
    gz::sim::EventManager &_eventMgr) override;

  void PreUpdate(
    const gz::sim::UpdateInfo &_info,
    gz::sim::EntityComponentManager &_ecm) override;

  void PostUpdate(
    const gz::sim::UpdateInfo &_info,
    const gz::sim::EntityComponentManager &_ecm) override;

private:
  struct Unit
  {
    std::string id;
    gz::sim::Entity model{gz::sim::kNullEntity};
    gz::sim::Entity left_joint{gz::sim::kNullEntity};
    gz::sim::Entity right_joint{gz::sim::kNullEntity};
    bool target_open{false};
    bool position_check_enabled{false};
  };

  void Load(
    const gz::sim::Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    gz::sim::EntityComponentManager &_ecm);

  void OnUnitCommand(
    std::size_t _index,
    const std::shared_ptr<std_srvs::srv::SetBool::Request> _request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> _response);

  static bool GetJointPositions(
    const Unit &_unit,
    const gz::sim::EntityComponentManager &_ecm,
    double &_left,
    double &_right);

  gz::sim::Entity world_entity_{gz::sim::kNullEntity};

  double door_speed_mps_{0.8};
  double door_travel_m_{1.8};
  double state_rate_hz_{10.0};
  double lock_delay_s_{0.3};
  double tolerance_m_{0.01};

  std::vector<Unit> units_;
  mutable std::mutex unit_mutex_;
  bool state_initialized_{false};
  double last_state_pub_time_{-1.0};
  double closed_since_{-1.0};
  bool last_state_{false};

  std::shared_ptr<rclcpp::Node> node_;
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr state_pub_;
  std::vector<rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr> services_;
};

}  // namespace gazebo
}  // namespace metro

#endif  // METRO_GAZEBO__DOOR_CONTROLLER_HPP_
