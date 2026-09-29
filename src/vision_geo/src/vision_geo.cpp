#include "vision_geo_.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<VisionGeoNode>();
  const std::string order_topic = node->get_parameter("order_topic").as_string();
  const bool wait_for_order     = node->get_parameter("wait_for_order").as_bool();
  const double order_timeout    = node->get_parameter("order_timeout").as_double();

  if (wait_for_order) {
    auto waiter = std::make_shared<rclcpp::Node>("vision_geo_wait");
    if (!waitCommand(waiter, "UAV-GO", order_topic, order_timeout)) {
      if (rclcpp::ok()) RCLCPP_ERROR(node->get_logger(), "No UAV-GO received, vision_geo exiting");
      rclcpp::shutdown();
      return 1;
    }
  }

  RCLCPP_INFO(node->get_logger(), "======================= VISION STARTED ===========================");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
