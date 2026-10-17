#include "vision_geo_.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<VisionGeoNode>();
  // waits for UAV-GO:VISION-... itself (wait_for_order), see VisionGeoNode::orderCallback
  RCLCPP_INFO(node->get_logger(), "======================= VISION STARTED ===========================");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
