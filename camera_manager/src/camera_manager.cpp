#include <rclcpp/rclcpp.hpp>
#include "camera_manager.hpp"
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>

// Pub: Node, Edges
// Sub: RS images.

CameraManager::CameraManager() : Node("camera_manager") {
  RCLCPP_INFO(this->get_logger(), "Camera Manager Node Initialized");
  camera_sub = this->create_subscription<sensor_msgs::msg::Image>("/camera/camera/color/image_raw", 10, std::bind(&CameraManager::image_callback, this, std::placeholders::_1));
}

void CameraManager::image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
{
  // Convert image to cv frame.
  cv::Mat frame;
  cv::Mat img = cv_bridge::toCvCopy(msg, "bgr8")->image;
  mFrame = img.clone();
  mFrameId ++;
}


void CameraManager::FeatureExtractor() {
  // Implement feature extraction logic using SIFT and ORB.
  cv::SIFT sift;
  std::vector<cv::KeyPoint> keypoints;
  cv::Mat descriptors;
  sift.detect(mFrame, keypoints);
  sift.compute(mFrame, keypoints, descriptors);
  mFeatureMap[mFrameId] = std::make_pair(keypoints, descriptors);
}


int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CameraManager>());
  rclcpp::shutdown();
  return 0;
}