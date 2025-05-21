#ifndef CAMERA_MANAGER_HPP
#define CAMERA_MANAGER_HPP
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.hpp>

using Image = sensor_msgs::msg::Image;

struct OpenCVImage {
  cv::Mat image;
  std::string encoding;
};

class CameraManager : public rclcpp::Node {
public:
  CameraManager();
  ~CameraManager() = default;

  void FeatureExtractor();
  void CameraPoseEstimation();
  void GraphBuilder();
  void LoopClosureDetector();

private: // ROS Components
  // Timer for camera image processing
  rclcpp::TimerBase::SharedPtr timer;
  // Subscriber to camera image
  rclcpp::Subscription<Image>::SharedPtr cameraSub;

private: // Member Variables
  // OpenCV Image
  OpenCVImage cvImage;

};
#endif // !CAMERA_MANAGER_HPP