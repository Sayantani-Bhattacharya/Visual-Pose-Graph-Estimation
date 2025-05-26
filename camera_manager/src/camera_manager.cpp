#include "camera_manager.hpp"

CameraManager::CameraManager() : Node("camera_manager") {
  RCLCPP_INFO(this->get_logger(), "Camera Manager Node Initialized");

  // Declare parameters
  this->timerFreq = this->declare_parameter("timer_frequency", 10.0f); // [Hz]
  // Get params from config file
  this->timerFreq = this->get_parameter("timer_frequency").as_double();

  // Create subscriber to camera info
  this->cameraInfoSub = this->create_subscription<CameraInfo>(
    "camera/camera/color/camera_info", 10,
    [this](const CameraInfo::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(this->frameMutex);
    this->cameraIntrinsics.K = cv::Mat(3, 3, CV_64F, const_cast<double*>(msg->k.data())).clone();
    this->cameraIntrinsics.D = cv::Mat(msg->d.size(), 1, CV_64F, const_cast<double*>(msg->d.data())).clone();
    this->collectedCameraInfo = true;
    RCLCPP_INFO(this->get_logger(), "Camera Intrinsics Collected");
  }
  );

  // Create subscriber to camera image and convert to OpenCV Image
  this->cameraSub = this->create_subscription<Image>(
    "camera/camera/color/image_raw", 10,
    [this](const Image::SharedPtr msg) {
    static int frameID = 0;
    if (!this->collectedCameraInfo) {
      RCLCPP_WARN(this->get_logger(), "Camera Info not collected yet");
      return;
    }
    // Convert ROS Image to OpenCV Image
    cv::Mat image = cv_bridge::toCvCopy(msg, "bgr8")->image;
    Frame f;
    f.stamp = this->now();
    f.frameID = frameID++;
    f.image = image;
    // Create a scope to lock the mutex before accessing the frame queue
    {
      std::lock_guard<std::mutex> lock(this->frameMutex);
      f.K = this->cameraIntrinsics.K.clone();
      f.D = this->cameraIntrinsics.D.clone();
      this->frameQueue.push(std::move(f));
    }
  }
  );

  // TODO: Add support for stereo cameras by subscribing to left and right images.

  // Timer for processing camera images
  this->timer = this->create_wall_timer(
    std::chrono::milliseconds(static_cast<int>(1000.0f / this->timerFreq)),
    std::bind(&CameraManager::timerCallback, this)
  );
}

void CameraManager::timerCallback() {
  Frame currentFrame;
  {
    std::lock_guard<std::mutex> lock(this->frameMutex);
    if (!this->frameQueue.empty()) {
      currentFrame = std::move(this->frameQueue.front());
      this->frameQueue.pop();
    }
  }
  Feature features = this->FeatureExtractor(currentFrame);
  Edge odomEdge = this->MonocularCameraPoseEstimation(features);
  Edge loopConstraints = this->LoopClosureDetector();
  this->GraphBuilder(odomEdge, loopConstraints);
  this->VisualizeGraph();
}

Feature CameraManager::FeatureExtractor(const Frame& frame) {
  // Implement feature extraction logic using SIFT and ORB.
  // TODO: SIFT is slow but accurate than ORB, may need to switch to ORB for real-time applications.
  cv::Ptr<cv::Feature2D> extractor = cv::SIFT::create();
  Feature feature;
  feature.frameID = frame.frameID;
  featureMap[feature.frameID] = feature; 

  // TODO: Add check for empty image.
  // TODO: Add left and right image support for stereo cameras.
  extractor->detectAndCompute(
    frame.image, cv::noArray(), feature.keypoints, feature.descriptors
  );
  return feature;
}

Edge CameraManager::MonocularCameraPoseEstimation(const Feature& feature) {
  // Implement camera pose estimation logic.
  if (featureMap.find(feature.frameID - 1) == featureMap.end()) {
      RCLCPP_WARN(this->get_logger(), "Previous frame not found for pose estimation.");
      return Edge(); // Return empty edge
  }
  Feature prevFeature = featureMap[feature.frameID - 1];

  cv::Mat K = this->cameraIntrinsics.K;
  cv::Mat D = this->cameraIntrinsics.D;  
  
  Edge edge;
  edge.fromID = feature.frameID;
  edge.toID = feature.frameID + 1; // Example increment


  // Match descriptors
  cv::BFMatcher matcher(cv::NORM_L2);
  std::vector<cv::DMatch> matches;
  matcher.match(prevFeature.descriptors, feature.descriptors, matches);

  // Filter good matches
  double max_dist = 0; double min_dist = 100;
  for (int i = 0; i < matches.size(); i++) {
      double dist = matches[i].distance;
      if (dist < min_dist) min_dist = dist;
      if (dist > max_dist) max_dist = dist;
  }
  std::vector<cv::DMatch> good_matches;
  for (int i = 0; i < matches.size(); i++) {
      if (matches[i].distance <= std::max(2 * min_dist, 30.0)) {
          good_matches.push_back(matches[i]);
      }
  }

  // Extract matched points
  std::vector<cv::Point2f> pts1, pts2;
  for (size_t i = 0; i < good_matches.size(); i++) {
      pts1.push_back(prevFeature.keypoints[good_matches[i].queryIdx].pt);
      pts2.push_back(feature.keypoints[good_matches[i].trainIdx].pt);
  }

  if (pts1.size() < 5 || pts2.size() < 5) {
      edge.relativePose = cv::Mat::eye(4, 4, CV_64F); // Not enough points
      return edge;
  }

  // Estimate essential matrix
  cv::Mat mask;
  cv::Mat E = cv::findEssentialMat(pts1, pts2, K, cv::RANSAC, 0.999, 1.0, mask);

  // Recover pose
  cv::Mat R, t;
  int inliers = cv::recoverPose(E, pts1, pts2, K, R, t, mask);

  // Build 4x4 transformation matrix
  edge.relativePose = cv::Mat::eye(4, 4, CV_64F);
  R.copyTo(edge.relativePose(cv::Rect(0, 0, 3, 3)));
  t.copyTo(edge.relativePose(cv::Rect(3, 0, 1, 3)));

  return edge;
}

Edge CameraManager::LoopClosureDetector() {
  // Implement loop closure detection logic.
  Edge edge;
  edge.fromID = -1; // Placeholder
  edge.toID = -1; // Placeholder
  return edge;
}

void CameraManager::GraphBuilder(const Edge& estimatedPose, const Edge& loopConstraints) {
  // Implement graph building logic.
}

void CameraManager::VisualizeGraph() {
  // Implement graph visualization logic.
}

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CameraManager>());
  rclcpp::shutdown();
  return 0;
}