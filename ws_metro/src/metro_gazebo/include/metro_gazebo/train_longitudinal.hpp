// TrainLongitudinal: Gazebo model plugin for the 1-D train motion model.
//
// Inputs  : /metro/train/command  (gz.msgs.Twist, only linear.x is used)
// Outputs : /metro/train/odom     (gz.msgs.Odometry, 50 Hz, frame "station")
// Rules   : the plugin is an actuator only.  R1/R3/R5 train-command arbitration
//           is performed by the interlock node before the command reaches this
//           plugin through ros_gz_bridge.
//
// The train is a prismatic body constrained to the world x axis; no differential
// drive is used.  Gazebo publishes gz.msgs.Odometry and ros_gz_bridge converts it
// to nav_msgs/Odometry on the ROS side.
#ifndef METRO_GAZEBO__TRAIN_LONGITUDINAL_HPP_
#define METRO_GAZEBO__TRAIN_LONGITUDINAL_HPP_

#include <atomic>
#include <string>

#include <gz/sim/System.hh>
#include <gz/sim/Entity.hh>
#include <gz/transport/Node.hh>
#include <gz/msgs/odometry.pb.h>
#include <gz/msgs/twist.pb.h>

namespace metro
{
namespace gazebo
{

class TrainLongitudinal
  : public gz::sim::System,
    public gz::sim::ISystemConfigure,
    public gz::sim::ISystemPreUpdate,
    public gz::sim::ISystemPostUpdate
{
public:
  TrainLongitudinal() = default;
  ~TrainLongitudinal() override = default;

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
  void Load(
    const gz::sim::Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    gz::sim::EntityComponentManager &_ecm);

  void OnCommand(const gz::msgs::Twist &_msg);

  gz::sim::Entity model_entity_{gz::sim::kNullEntity};
  gz::sim::Entity joint_entity_{gz::sim::kNullEntity};

  std::string odom_topic_{"/metro/train/odom"};
  std::string command_topic_{"/metro/train/command"};
  double odom_rate_hz_{50.0};
  double max_speed_mps_{20.0};
  double max_accel_mps2_{1.0};
  double max_decel_mps2_{2.0};
  double rail_y_m_{6.65};

  double initial_x_m_{0.0};
  bool initial_pose_initialized_{false};
  double measured_velocity_mps_{0.0};
  double last_odom_time_s_{-1.0};

  std::atomic<double> commanded_speed_mps_{0.0};

  gz::transport::Node transport_node_;
  gz::transport::Node::Publisher odom_publisher_;
};

}  // namespace gazebo
}  // namespace metro

#endif  // METRO_GAZEBO__TRAIN_LONGITUDINAL_HPP_
