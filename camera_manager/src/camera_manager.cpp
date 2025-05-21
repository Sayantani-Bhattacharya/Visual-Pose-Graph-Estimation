#include "camera_manager.hpp"

CameraManager::CameraManager() : Node("camera_manager") {
  RCLCPP_INFO(this->get_logger(), "Camera Manager Node Initialized");

  // Declare parameters
  this->timerFreq = this->declare_parameter("timer_frequency", 10.0f); // [Hz]
  // Get params from config file
  this->timerFreq = this->get_parameter("timer_frequency").as_double();

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

  // Timer for processing camera images
  this->timer = this->create_wall_timer(
    std::chrono::seconds(static_cast<int>(1.0 / this->timerFreq)),
    std::bind(&CameraManager::timerCallback, this)
  );
}

void CameraManager::timerCallback() {
  // Process camera image
  if (!this->cvImage.image.empty()) {
    this->FeatureExtractor();
    this->CameraPoseEstimation();
    this->GraphBuilder();
    this->LoopClosureDetector();
  }
}

void CameraManager::FeatureExtractor() {
  // Implement feature extraction logic using SIFT and ORB.
  cv::SIFT sift;
  std::vector<cv::KeyPoint> keypoints;
  cv::Mat descriptors;
  sift.detect(this->cvImage.image, keypoints);
  sift.compute(this->cvImage.image, keypoints, descriptors);
  this->featureMap[this->cvImage.frameID] = std::make_pair(keypoints, descriptors);
}

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CameraManager>());
  rclcpp::shutdown();
  return 0;
}