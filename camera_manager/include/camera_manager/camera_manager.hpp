#ifndef CAMERA_MANAGER_HPP
#define CAMERA_MANAGER_HPP
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <opencv2/opencv.hpp>

class CameraManager : public rclcpp::Node {
public:
  CameraManager();
  ~CameraManager() = default;
  void FeatureExtractor();
  void CameraPoseEstimation();
  void GraphBuilder();
  // void CameraPoseGraphOptimization();
  // void LoopClosureDetector();


private:
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr camera_sub;

  // cv::frame mFrame;


  void image_callback(const sensor_msgs::msg::Image::SharedPtr msg);

};
#endif // !CAMERA_MANAGER_HPP