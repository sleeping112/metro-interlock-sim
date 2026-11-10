// PlatformIntrusionDetector: model-based platform-edge intrusion monitor.
//
// Inputs  : positions of passenger_XX models in the Gazebo world.
// Outputs : /metro/platform/intrusion (std_msgs/Bool, 10 Hz)
//
// A passenger is considered an intrusion when its body enters the danger strip
// between the tactile strip and the platform edge while the train doors are in
// use.  This is a deterministic perception surrogate for the demo; a camera or
// depth detector can be added later without changing the topic.
#include <algorithm>
#include <chrono>
#include <cmath>
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
  stream.width(2);
  stream.fill('0');
  stream << _value;
  return stream.str();
}

}  // namespace

class PlatformIntrusionDetector
  : public gz::sim::System,
    public gz::sim::ISystemConfigure,
    public gz::sim::ISystemPostUpdate
{
public:
  void Configure(
    const gz::sim::Entity &,
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
      if (_sdf->HasElement("danger_y_min_m")) {
        danger_y_min_ = _sdf->Get<double>("danger_y_min_m");
      }
      if (_sdf->HasElement("danger_y_max_m")) {
        danger_y_max_ = _sdf->Get<double>("danger_y_max_m");
      }
      if (_sdf->HasElement("publish_rate_hz")) {
        publish_rate_hz_ = std::max(0.5, _sdf->Get<double>("publish_rate_hz"));
      }
    }

    for (int index = 1; index <= passenger_count_; ++index) {
      const auto entity = _ecm.EntityByComponents(
        gz::sim::components::Model(),
        gz::sim::components::Name(passenger_prefix_ + TwoDigit(index)));
      if (entity != gz::sim::kNullEntity) {
        passengers_.push_back(entity);
      }
    }

    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
    node_ = std::make_shared<rclcpp::Node>("metro_platform_intrusion");
    publisher_ = node_->create_publisher<std_msgs::msg::Bool>(
      "/metro/platform/intrusion", rclcpp::QoS(10).reliable());

    std::cout << "[PlatformIntrusionDetector] passengers=" << passengers_.size()
              << " danger_y=[" << danger_y_min_ << ", " << danger_y_max_
              << "] rate=" << publish_rate_hz_ << " Hz" << std::endl;
  }

  void PostUpdate(
    const gz::sim::UpdateInfo &_info,
    const gz::sim::EntityComponentManager &_ecm) override
  {
    if (_info.paused) {
      return;
    }
    const double now = std::chrono::duration<double>(_info.simTime).count();
    if (last_publish_s_ >= 0.0 && (now - last_publish_s_) < 1.0 / publish_rate_hz_) {
      return;
    }
    last_publish_s_ = now;

    bool intrusion = false;
    for (const auto entity : passengers_) {
      const auto pose = gz::sim::worldPose(entity, _ecm);
      const double x = pose.Pos().X();
      const double y = pose.Pos().Y();
      const double z = pose.Pos().Z();
      if (z > -1.0 && z < 2.2 && x > -66.0 && x < 66.0 &&
          y > danger_y_min_ && y < danger_y_max_)
      {
        intrusion = true;
        break;
      }
    }

    if (node_) {
      std_msgs::msg::Bool message;
      message.data = intrusion;
      publisher_->publish(message);
    }

    if (intrusion != last_intrusion_) {
      std::cout << "[PlatformIntrusionDetector] platform intrusion -> "
                << (intrusion ? "true" : "false") << " at t=" << now << std::endl;
      last_intrusion_ = intrusion;
    }
  }

private:
  std::string passenger_prefix_{"passenger_"};
  int passenger_count_{12};
  double danger_y_min_{6.55};
  double danger_y_max_{7.15};
  double publish_rate_hz_{10.0};
  double last_publish_s_{-1.0};
  bool last_intrusion_{false};
  std::vector<gz::sim::Entity> passengers_;

  std::shared_ptr<rclcpp::Node> node_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr publisher_;
};

}  // namespace gazebo
}  // namespace metro

GZ_ADD_PLUGIN(
  metro::gazebo::PlatformIntrusionDetector,
  gz::sim::System,
  metro::gazebo::PlatformIntrusionDetector::ISystemConfigure,
  metro::gazebo::PlatformIntrusionDetector::ISystemPostUpdate)

GZ_ADD_PLUGIN_ALIAS(
  metro::gazebo::PlatformIntrusionDetector,
  "metro::gazebo::PlatformIntrusionDetector")
