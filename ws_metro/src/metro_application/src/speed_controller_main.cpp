#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "metro_application/train_speed_controller_node.hpp"

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<metro::application::TrainSpeedControllerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
