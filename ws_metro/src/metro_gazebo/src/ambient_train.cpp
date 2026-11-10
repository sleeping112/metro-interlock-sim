// AmbientTrain: visual-only opposite-direction train.
//
// The model is static and collision-free; the plugin moves its model pose along
// the -y track to add station atmosphere.  It has no ROS interface and never
// affects the interlocked train or PSD logic.
#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>

#include <gz/plugin/Register.hh>
#include <gz/sim/System.hh>
#include <gz/sim/components/Pose.hh>

namespace metro
{
namespace gazebo
{

class AmbientTrain
  : public gz::sim::System,
    public gz::sim::ISystemConfigure,
    public gz::sim::ISystemPreUpdate
{
public:
  void Configure(
    const gz::sim::Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    gz::sim::EntityComponentManager &_ecm,
    gz::sim::EventManager &) override
  {
    model_entity_ = _entity;
    if (_sdf) {
      if (_sdf->HasElement("loop_start_x")) {
        loop_start_x_ = _sdf->Get<double>("loop_start_x");
      }
      if (_sdf->HasElement("loop_end_x")) {
        loop_end_x_ = _sdf->Get<double>("loop_end_x");
      }
      if (_sdf->HasElement("speed_mps")) {
        speed_mps_ = std::max(0.1, _sdf->Get<double>("speed_mps"));
      }
      if (_sdf->HasElement("period_s")) {
        period_s_ = std::max(1.0, _sdf->Get<double>("period_s"));
      }
    }
    std::cout << "[AmbientTrain] start_x=" << loop_start_x_
              << " end_x=" << loop_end_x_
              << " speed_mps=" << speed_mps_
              << " period_s=" << period_s_ << std::endl;
  }

  void PreUpdate(
    const gz::sim::UpdateInfo &_info,
    gz::sim::EntityComponentManager &_ecm) override
  {
    if (_info.paused) {
      return;
    }
    const double now = std::chrono::duration<double>(_info.simTime).count();
    const double distance = loop_end_x_ - loop_start_x_;
    const double cycle = std::fmod(now * speed_mps_, distance);
    const double x = loop_start_x_ + cycle;
    const gz::math::Pose3d pose(x, -8.65, 2.30, 0.0, 0.0, 0.0);
    _ecm.SetComponentData<gz::sim::components::Pose>(model_entity_, pose);
  }

private:
  gz::sim::Entity model_entity_{gz::sim::kNullEntity};
  double loop_start_x_{-200.0};
  double loop_end_x_{200.0};
  double speed_mps_{9.0};
  double period_s_{55.0};
};

}  // namespace gazebo
}  // namespace metro

GZ_ADD_PLUGIN(
  metro::gazebo::AmbientTrain,
  gz::sim::System,
  metro::gazebo::AmbientTrain::ISystemConfigure,
  metro::gazebo::AmbientTrain::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(
  metro::gazebo::AmbientTrain,
  "metro::gazebo::AmbientTrain")
