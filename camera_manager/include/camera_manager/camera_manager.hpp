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
  void image_callback(const sensor_msgs::msg::Image::SharedPtr msg);



private:

  // Global variables
  
  // Real-time updated CV frame.
  cv::Mat mFrame;
  int mFrameId = 0;
  
  std::map<int, std::pair<std::vector<cv::KeyPoint>, cv::Mat>> mFeatureMap;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr camera_sub;



};
#endif // !CAMERA_MANAGER_HPP