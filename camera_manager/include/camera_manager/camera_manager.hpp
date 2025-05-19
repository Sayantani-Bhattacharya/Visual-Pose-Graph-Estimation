#ifndef CAMERA_MANAGER_HPP
#define CAMERA_MANAGER_HPP
#include <rclcpp/rclcpp.hpp>


class CameraManager : public rclcpp::Node {
public:
  CameraManager();
  ~CameraManager() = default;
};

#endif // !CAMERA_MANAGER_HPP