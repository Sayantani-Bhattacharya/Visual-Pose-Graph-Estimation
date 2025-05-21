#include <rclcpp/rclcpp.hpp>
#include "camera_manager.hpp"

CameraManager::CameraManager() : Node("camera_manager") {
  RCLCPP_INFO(this->get_logger(), "Camera Manager Node Initialized");

  camera_sub = this->create_subscription<sensor_msgs::msg::Image>("camera/image", 10, std::bind(&CameraManager::image_callback, this, std::placeholders::_1));

}

void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
  // RCLCPP_INFO(this->get_logger(), "Received image with width: %d, height: %d", msg->width, msg->height);
  // /convert msg to cv frame.
}



int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CameraManager>());
  rclcpp::shutdown();
  return 0;
}