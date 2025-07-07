#include "slam/relocalization.h"

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  
  auto node = std::make_shared<kiss_matcher::RelocalizationNode>();
  
  RCLCPP_INFO(node->get_logger(), "Starting KISS-Matcher relocalization node...");
  
  try {
    rclcpp::spin(node);
  } catch (const std::exception& e) {
    RCLCPP_ERROR(node->get_logger(), "Exception in relocalization node: %s", e.what());
  }
  
  rclcpp::shutdown();
  return 0;
}