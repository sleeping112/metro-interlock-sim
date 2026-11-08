// Longitudinal speed governor shared by the interlock (automatic station stop)
// and the stand-alone train_speed_controller_node.
//
// Inputs : x [m], v [m/s], auto_run flag, manual speed.
// Output : desired longitudinal velocity [m/s], never negative (the station is
//          always approached in the +x direction).
//
// The curve uses a conservative braking deceleration and a small approach
// margin so that the 1.0 m/s^2 governor can be tracked by the plugin's 2.0 m/s^2
// service brake without overshooting the +/-0.5 m stop window.
#ifndef METRO_APPLICATION__TRAIN_SPEED_CONTROLLER_HPP_
#define METRO_APPLICATION__TRAIN_SPEED_CONTROLLER_HPP_

#include <algorithm>
#include <cmath>

namespace metro
{
namespace application
{

struct TrainSpeedConfig
{
  double x_stop_m{0.0};
  double cruise_speed_mps{8.0};
  double brake_decel_mps2{1.0};
  double approach_margin_m{0.5};
  double max_speed_mps{20.0};
};

class TrainSpeedController
{
public:
  explicit TrainSpeedController(const TrainSpeedConfig &_config)
  : config_(_config) {}

  double Compute(double _x, double _v, bool _auto_run, double _manual_speed) const
  {
    if (!_auto_run) {
      return std::clamp(_manual_speed, -config_.max_speed_mps, config_.max_speed_mps);
    }

    const double remaining = config_.x_stop_m - _x;
    if (remaining <= 0.0 || std::abs(_v) < 1e-4 && remaining < 0.05) {
      return 0.0;
    }

    // Do not command a speed larger than the conservative braking curve allows.
    const double usable = std::max(0.0, remaining - config_.approach_margin_m);
    const double allowed = std::sqrt(2.0 * config_.brake_decel_mps2 * usable);
    const double target = std::min(config_.cruise_speed_mps, allowed);
    return std::clamp(target, 0.0, config_.max_speed_mps);
  }

  const TrainSpeedConfig &Config() const {return config_;}

private:
  TrainSpeedConfig config_;
};

}  // namespace application
}  // namespace metro

#endif  // METRO_APPLICATION__TRAIN_SPEED_CONTROLLER_HPP_
