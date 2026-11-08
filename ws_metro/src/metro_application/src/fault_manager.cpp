#include "metro_application/fault_manager.hpp"

#include <utility>

namespace metro
{
namespace application
{

FaultManager::FaultManager(
  rclcpp::Clock::SharedPtr _clock,
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr _publisher,
  double _hold_s,
  double _suppression_s)
: clock_(std::move(_clock)),
  publisher_(std::move(_publisher)),
  hold_(rclcpp::Duration::from_seconds(_hold_s)),
  suppression_(rclcpp::Duration::from_seconds(_suppression_s))
{
}

bool FaultManager::Raise(const std::string &_code)
{
  if (_code.empty()) {
    return false;
  }

  const rclcpp::Time now = clock_->now();
  if (active_ && now < active_->until) {
    // The currently published code owns the full hold interval.  A different
    // code may arrive (e.g. R5 followed by R3 after emergency braking), but it
    // is queued until the current 1 s hold expires so subscribers always see a
    // stable code for the specified hold time.
    return false;
  }
  if (active_ && now >= active_->until) {
    Publish(empty_);
    active_.reset();
  }

  const auto previous = last_publish_.find(_code);
  if (previous != last_publish_.end() &&
      (now - previous->second) < suppression_)
  {
    // Suppressed for publication, but the caller still applies the safety
    // action from live conditions every cycle.
    return false;
  }

  Publish(_code);
  active_ = ActiveFault{_code, now + hold_};
  last_publish_[_code] = now;
  return true;
}

void FaultManager::Tick()
{
  if (!active_) {
    return;
  }

  const rclcpp::Time now = clock_->now();
  if (now >= active_->until) {
    Publish(empty_);
    active_.reset();
    return;
  }

  // Hold the code visible for the full hold interval.
  Publish(active_->code);
}

void FaultManager::Clear()
{
  if (active_) {
    Publish(empty_);
    active_.reset();
  }
}

const std::string &FaultManager::ActiveCode() const
{
  return active_ ? active_->code : empty_;
}

void FaultManager::Publish(const std::string &_code)
{
  std_msgs::msg::String message;
  message.data = _code;
  publisher_->publish(message);
}

}  // namespace application
}  // namespace metro
