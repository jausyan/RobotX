#include "vision_geo_.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VisionGeoNode>());
  rclcpp::shutdown();
  return 0;
}
