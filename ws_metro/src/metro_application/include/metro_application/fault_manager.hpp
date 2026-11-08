// FaultManager owns the /metro/interlock/fault publication lifecycle.
//
// Rules:
//   * a raised code is held (republished at the control rate) for hold_s;
//   * after hold_s an empty string is published to clear it;
//   * the same code is not published again within suppression_s.
//
// The actuator-lock decision is made from live conditions in InterlockNode, not
// from this publisher state, so a suppressed fault can never release traction.
#ifndef METRO_APPLICATION__FAULT_MANAGER_HPP_
#define METRO_APPLICATION__FAULT_MANAGER_HPP_

#include <optional>
#include <string>
#include <unordered_map>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

namespace metro
{
namespace application
{

class FaultManager
{
public:
  FaultManager(
    rclcpp::Clock::SharedPtr _clock,
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr _publisher,
    double _hold_s,
    double _suppression_s);

  bool Raise(const std::string &_code);
  void Tick();
  void Clear();

  const std::string &ActiveCode() const;

private:
  void Publish(const std::string &_code);

  rclcpp::Clock::SharedPtr clock_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::Duration hold_;
  rclcpp::Duration suppression_;

  struct ActiveFault
  {
    std::string code;
    rclcpp::Time until;
  };

  std::optional<ActiveFault> active_;
  std::unordered_map<std::string, rclcpp::Time> last_publish_;
  std::string empty_;
};

}  // namespace application
}  // namespace metro

#endif  // METRO_APPLICATION__FAULT_MANAGER_HPP_
