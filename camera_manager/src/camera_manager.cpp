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
    this->cvImageMutex.lock();
    this->cvImage.header = msg->header;
    // Set frame ID to a unique value based on timestamp [sec * 1000000 + nanosec / 1000]
    this->cvImage.frameID = (msg->header.stamp.sec * 1000000) + (msg->header.stamp.nanosec / 1000);
    this->cvImage.image = image;
    this->cvImage.encoding = msg->encoding;
    this->cvImageMutex.unlock();
  }
  );

  // Timer for processing camera images
  this->timer = this->create_wall_timer(
    std::chrono::seconds(static_cast<int>(1.0 / this->timerFreq)),
    std::bind(&CameraManager::timerCallback, this)
  );
}

void CameraManager::timerCallback() {
  this->cvImageMutex.lock();
  OpenCVImage currentImage = this->cvImage;
  this->cvImageMutex.unlock();
  if (!currentImage.image.empty()) {
    FeatureMap features = this->FeatureExtractor(currentImage);
    PoseConstraints estimatedPose = this->CameraPoseEstimation(features);
    PoseConstraints loopConstraints = this->LoopClosureDetector();
    this->GraphBuilder(estimatedPose, loopConstraints);
    this->VisualizeGraph();
  }
}

FeatureMap CameraManager::FeatureExtractor(const OpenCVImage& frame) {
  // Implement feature extraction logic using SIFT and ORB.
  cv::SIFT sift;
  std::vector<cv::KeyPoint> keypoints;
  cv::Mat descriptors;
  sift.detect(frame.image, keypoints);
  sift.compute(frame.image, keypoints, descriptors);
  this->featureMap[frame.frameID] = std::make_pair(keypoints, descriptors);
}

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CameraManager>());
  rclcpp::shutdown();
  return 0;
}