#include "camera_manager.hpp"

CameraManager::CameraManager() : Node("camera_manager") {
  RCLCPP_INFO(this->get_logger(), "Camera Manager Node Initialized");

  // Create subscriber to camera image and convert to OpenCV Image
  this->cameraSub = this->create_subscription<Image>(
    "camera/camera/color/image_raw", 10,
    [this](const Image::SharedPtr msg) {
    // Convert ROS Image to OpenCV Image
    cv::Mat image = cv_bridge::toCvCopy(msg, "bgr8")->image;
    this->cvImage.image = image;
    this->cvImage.encoding = msg->encoding;
  }
  );
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