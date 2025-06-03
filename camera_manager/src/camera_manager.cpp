#include "camera_manager.hpp"

CameraManager::CameraManager() : Node("camera_manager") {
  RCLCPP_INFO(this->get_logger(), "Camera Manager Node Initialized");

  // Declare parameters.
  this->timerFreq = this->declare_parameter("timer_frequency", 10.0); // [Hz] Timer for managing pose-graph
  this->useStereoCamera = this->declare_parameter("use_stereo_camera", true); // Use stereo camera by default
  this->stereoBaseline = this->declare_parameter("stereo_baseline", 0.05); // [m] Default baseline distance between stereo cameras
  // Get params from config file.
  this->timerFreq = this->get_parameter("timer_frequency").as_double();
  this->useStereoCamera = this->get_parameter("use_stereo_camera").as_bool();
  this->stereoBaseline = this->get_parameter("stereo_baseline").as_double();

  // Setup Path publisher for camera trajectory
  this->cameraEstimatePathPub = this->create_publisher<Path>("camera_trajectory", 10);
  this->cameraPoseEstimatePath.header.frame_id = "camera_link";

  // Setup Image publisher for feature visualization
  this->featureImagePub = this->create_publisher<ImageMsg>("camera/feature_image", 10);

  // Setup tf broadcaster
  this->tfBroadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(this);


  // Create subscribers for stereo and monocular cameras
  if (this->useStereoCamera) {
    // Setup left and right camera subscribers
    this->leftCameraSub = std::make_shared<message_filters::Subscriber<ImageMsg>>(this, "camera/camera/infra1/image_rect_raw");
    this->leftCameraInfoSub = std::make_shared<message_filters::Subscriber<CameraInfoMsg>>(this, "camera/camera/infra1/camera_info");
    this->rightCameraSub = std::make_shared<message_filters::Subscriber<ImageMsg>>(this, "camera/camera/infra2/image_rect_raw");
    this->rightCameraInfoSub = std::make_shared<message_filters::Subscriber<CameraInfoMsg>>(this, "camera/camera/infra2/camera_info");
    // Synchronize stereo camera messages
    this->stereoSync = std::make_shared<message_filters::Synchronizer<StereoSyncPolicy>>(
      StereoSyncPolicy(10), *leftCameraSub, *leftCameraInfoSub, *rightCameraSub, *rightCameraInfoSub
    );
    this->stereoSync->registerCallback(
      std::bind(
        &CameraManager::synchronizedStereoCallback,
        this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4
      )
    );
  } else {
    // Setup monocular camera subscriber
    this->cameraSub = std::make_shared<message_filters::Subscriber<ImageMsg>>(this, "camera/camera/color/image_raw");
    this->cameraInfoSub = std::make_shared<message_filters::Subscriber<CameraInfoMsg>>(this, "camera/camera/color/camera_info");
    // Synchronize monocular camera messages
    this->monoSync = std::make_shared<message_filters::Synchronizer<MonoSyncPolicy>>(
      MonoSyncPolicy(10), *cameraSub, *cameraInfoSub
    );
    this->monoSync->registerCallback(
      std::bind(
        &CameraManager::synchronizedMonocularCallback,
        this, std::placeholders::_1, std::placeholders::_2
      )
    );
  }

  // Initialize current estimate
  this->currentPose = cv::Mat::eye(4, 4, CV_64F); // SE(3) identity matrix

  // Initialize PoseGraph
  this->initializePoseGraph();

  // Publish a static transform at the origin of the world frame
  geometry_msgs::msg::TransformStamped worldTransform;
  worldTransform.header.stamp = this->now();
  worldTransform.header.frame_id = "world";
  worldTransform.child_frame_id = "camera_link";
  worldTransform.transform.translation.x = 0.0;
  worldTransform.transform.translation.y = 0.0;
  worldTransform.transform.translation.z = 0.0;
  worldTransform.transform.rotation.x = 0.0;
  worldTransform.transform.rotation.y = 0.0;
  worldTransform.transform.rotation.z = 0.0;
  worldTransform.transform.rotation.w = 1.0;
  this->tfBroadcaster->sendTransform(worldTransform);

  // Setup timer for pose-graph management and visualization
  this->timer = this->create_wall_timer(
    std::chrono::milliseconds(static_cast<int>(1000 / this->timerFreq)),
    std::bind(&CameraManager::timerCallback, this)
  );
}

void CameraManager::timerCallback() {

}

void CameraManager::synchronizedStereoCallback(
  const sensor_msgs::msg::Image::ConstSharedPtr left_image_msg,
  const sensor_msgs::msg::CameraInfo::ConstSharedPtr left_info_msg,
  const sensor_msgs::msg::Image::ConstSharedPtr right_image_msg,
  const sensor_msgs::msg::CameraInfo::ConstSharedPtr right_info_msg) {
  static int frameID = 0;
  Frame currentLeftFrame;
  Frame currentRightFrame;

  // Convert ROS Image to OpenCV Image
  currentLeftFrame.image = cv_bridge::toCvCopy(left_image_msg, "bgr8")->image;
  currentRightFrame.image = cv_bridge::toCvCopy(right_image_msg, "bgr8")->image;

  // Extract intrinsics
  currentLeftFrame.intrinsics.K = cv::Mat(3, 3, CV_64F, const_cast<double*>(left_info_msg->k.data())).clone();
  currentLeftFrame.intrinsics.D = cv::Mat(left_info_msg->d.size(), 1, CV_64F, const_cast<double*>(left_info_msg->d.data())).clone();
  currentRightFrame.intrinsics.K = cv::Mat(3, 3, CV_64F, const_cast<double*>(right_info_msg->k.data())).clone();
  currentRightFrame.intrinsics.D = cv::Mat(right_info_msg->d.size(), 1, CV_64F, const_cast<double*>(right_info_msg->d.data())).clone();
  this->leftCameraIntrinsics.K = currentLeftFrame.intrinsics.K.clone();
  this->leftCameraIntrinsics.D = currentLeftFrame.intrinsics.D.clone();
  this->rightCameraIntrinsics.K = currentRightFrame.intrinsics.K.clone();
  this->rightCameraIntrinsics.D = currentRightFrame.intrinsics.D.clone();
  // Assign timestamps and frame IDs
  currentLeftFrame.stamp = left_image_msg->header.stamp;
  currentRightFrame.stamp = right_image_msg->header.stamp;
  currentLeftFrame.frameID = frameID;
  currentRightFrame.frameID = frameID;
  frameID++; // Increment for next set of frames

  if (currentLeftFrame.image.empty() || currentRightFrame.image.empty() || currentLeftFrame.intrinsics.K.empty()) {
    RCLCPP_WARN(this->get_logger(), "Received empty stereo images or camera info.");
  }

  // 1. Feature Extraction
  StereoFeature currentStereoFeature = this->StereoFeatureExtractor(currentLeftFrame, currentRightFrame);

  // Create a copy of the left image with overlaid features for visualization
  cv::Mat features;
  currentLeftFrame.image.copyTo(features);
  if (!currentStereoFeature.leftKeypoints.empty()) {
    cv::drawKeypoints(
      currentLeftFrame.image, currentStereoFeature.leftKeypoints, features,
      cv::Scalar(0, 255, 0), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS
    );
  }
  // Create ROS Image message to publish
  auto header = std_msgs::msg::Header();
  header.stamp = this->now();
  header.frame_id = "camera_link";
  auto featureImageMsg = cv_bridge::CvImage(
    header, "bgr8", features
  ).toImageMsg();
  // Publish the feature image
  this->featureImagePub->publish(*featureImageMsg);

  // Update previous features if we have not processed any frames yet
  if (this->previousStereoFeature.frameID == 0 && currentStereoFeature.frameID > 0) {
    this->previousStereoFeature = currentStereoFeature;
    return;
  }

  // 2. Odometry (Pose) Estimation 
  Edge odomEdge = this->StereoCameraPoseEstimation(currentStereoFeature);
  this->previousStereoFeature = currentStereoFeature; // Update previous feature for next iteration

  // 3. Pose-Graph Management
  if (!odomEdge.relativePose.empty()) {
    this->addEdge(odomEdge);
    // Update current pose estimate
    {
      std::lock_guard<std::mutex> lock(this->poseMutex);
      this->currentPose = odomEdge.relativePose * this->currentPose;
    }
    // Visualize the current camera pose
    this->UpdateCameraPoseVisualization();
  }

  // 4. Loop-Closure Detection
  // TODO: This should be managed carefully, in the timer or separate thread
  // For now this can be left out as we implement the rest of the pipeline
}

void CameraManager::synchronizedMonocularCallback(
  const sensor_msgs::msg::Image::ConstSharedPtr image_msg,
  const sensor_msgs::msg::CameraInfo::ConstSharedPtr info_msg) {
  static int frameID = 0;
  Frame currentFrame;
  // Convert ROS Image to OpenCV Image
  currentFrame.image = cv_bridge::toCvCopy(image_msg, "bgr8")->image;

  // Extract intrinsics
  currentFrame.intrinsics.K = cv::Mat(3, 3, CV_64F, const_cast<double*>(info_msg->k.data())).clone();
  currentFrame.intrinsics.D = cv::Mat(info_msg->d.size(), 1, CV_64F, const_cast<double*>(info_msg->d.data())).clone();
  this->cameraIntrinsics.K = currentFrame.intrinsics.K.clone();
  this->cameraIntrinsics.D = currentFrame.intrinsics.D.clone();
  currentFrame.stamp = image_msg->header.stamp;
  currentFrame.frameID = frameID++; // Increment frame ID
  if (currentFrame.image.empty() || currentFrame.intrinsics.K.empty()) {
    RCLCPP_WARN(this->get_logger(), "Received empty monocular image or camera info.");
  }

  // 1. Feature Extraction
  Feature currentFeature = this->MonocularFeatureExtractor(currentFrame);

  // Create a copy of the image with overlaid features for visualization
  cv::Mat features;
  currentFrame.image.copyTo(features);
  if (!currentFeature.keypoints.empty()) {
    cv::drawKeypoints(
      currentFrame.image, currentFeature.keypoints, features,
      cv::Scalar(0, 255, 0), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS
    );
  }
  // Create ROS Image message to publish
  auto header = std_msgs::msg::Header();
  header.stamp = this->now();
  header.frame_id = "camera_link";
  auto featureImageMsg = cv_bridge::CvImage(
    header, "bgr8", features
  ).toImageMsg();
  // Publish the feature image
  this->featureImagePub->publish(*featureImageMsg);

  // Update previous features if we have not processed any frames yet
  if (this->previousFeature.frameID == 0 && currentFeature.frameID > 0) {
    this->previousFeature = currentFeature;
    return;
  }

  // 2. Odometry (Pose) Estimation
  Edge odomEdge = this->MonocularCameraPoseEstimation(currentFeature);
  this->previousFeature = currentFeature; // Update previous feature for next iteration

  // 3. Pose-Graph Management
  if (!odomEdge.relativePose.empty()) {
    this->addEdge(odomEdge);
    // Update current pose estimate
    {
      std::lock_guard<std::mutex> lock(this->poseMutex);
      this->currentPose = odomEdge.relativePose * this->currentPose;
    }
    // Visualize the current camera pose
    this->UpdateCameraPoseVisualization();
  }

  // 4. Loop-Closure Detection
  // TODO: This should be managed carefully, in the timer or separate thread
  // For now this can be left out as we implement the rest of the pipeline
}

Feature CameraManager::MonocularFeatureExtractor(const Frame& frame) {
  // TODO: SIFT is slow but accurate than ORB, may need to switch to ORB for real-time applications.
  //TODO: Parameterize the feature extractor (SIFT, ORB, etc.) and descriptor matcher
  cv::Ptr<cv::Feature2D> extractor = cv::SIFT::create();
  Feature feature;
  feature.frameID = frame.frameID;
  if (frame.image.empty()) {
    RCLCPP_WARN(this->get_logger(), "[FeatureExtractor] Received empty image for frame ID %d", frame.frameID);
    return feature;
  }
  extractor->detectAndCompute(
    frame.image, cv::noArray(), feature.keypoints, feature.descriptors
  );
  return feature;
}

StereoFeature CameraManager::StereoFeatureExtractor(const Frame& leftFrame, const Frame& rightFrame) {
  //TODO: Parameterize the feature extractor (SIFT, ORB, etc.) and descriptor matcher
  StereoFeature stereoFeature;
  stereoFeature.frameID = leftFrame.frameID;

  if (leftFrame.image.empty() || rightFrame.image.empty()) {
    RCLCPP_WARN(this->get_logger(), "[StereoFeatureExtractor] Received empty images for frame ID %d", leftFrame.frameID);
    return stereoFeature;
  }

  cv::Ptr<cv::Feature2D> extractor = cv::SIFT::create();
  extractor->detectAndCompute(leftFrame.image, cv::noArray(), stereoFeature.leftKeypoints, stereoFeature.leftDescriptors);
  extractor->detectAndCompute(rightFrame.image, cv::noArray(), stereoFeature.rightKeypoints, stereoFeature.rightDescriptors);

  return stereoFeature;
}

Edge CameraManager::MonocularCameraPoseEstimation(const Feature& newFeature) {
  // This function will only be called if there's a valid previous feature
  cv::Mat K = this->cameraIntrinsics.K;
  cv::Mat D = this->cameraIntrinsics.D;

  Edge edge;
  edge.fromID = newFeature.frameID;
  edge.toID = newFeature.frameID + 1;

  // Match descriptors
  cv::BFMatcher matcher(cv::NORM_L2);
  std::vector<cv::DMatch> matches;
  matcher.match(previousFeature.descriptors, newFeature.descriptors, matches);

  // Filter good matches
  double max_dist = 0;
  double min_dist = 100;
  for (unsigned int i = 0; i < matches.size(); i++) {
    double dist = matches[i].distance;
    if (dist < min_dist) min_dist = dist;
    if (dist > max_dist) max_dist = dist;
  }
  std::vector<cv::DMatch> good_matches;
  for (unsigned int i = 0; i < matches.size(); i++) {
    if (matches[i].distance <= std::max(2 * min_dist, 30.0)) {
      good_matches.push_back(matches[i]);
    }
  }

  // Extract matched points
  std::vector<cv::Point2f> pts1, pts2;
  for (size_t i = 0; i < good_matches.size(); i++) {
    pts1.push_back(previousFeature.keypoints[good_matches[i].queryIdx].pt);
    pts2.push_back(newFeature.keypoints[good_matches[i].trainIdx].pt);
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

Edge CameraManager::StereoCameraPoseEstimation(const StereoFeature& newFeature) {
  // This function will only be called if there's a valid previous stereo feature

  // Create the edge between the previous and current stereo features
  Edge edge;
  edge.fromID = previousStereoFeature.frameID;
  edge.toID = newFeature.frameID;

  // Match descriptors between previous and current left images
  cv::BFMatcher matcher(cv::NORM_L2);
  std::vector<cv::DMatch> matches;
  matcher.match(previousStereoFeature.leftDescriptors, newFeature.leftDescriptors, matches);

  // Filter good matches 
  double min_dist = 100;
  for (const auto& m : matches) min_dist = std::min(min_dist, (double)m.distance);
  std::vector<cv::DMatch> good_matches;
  for (const auto& m : matches) {
    if (m.distance <= std::max(2 * min_dist, 30.0)) good_matches.push_back(m);
  }
  // Triangulate 3D points in previous and current frames
  std::vector<cv::Point3f> pts3d_prev, pts3d_curr;
  std::vector<cv::Point2f> pts2d_curr;
  const cv::Mat K = this->leftCameraIntrinsics.K; // 3x3 camera matrix
  for (const auto& m : good_matches) {
    // Get left/right keypoints for triangulation
    cv::Point2f kpL_prev = previousStereoFeature.leftKeypoints[m.queryIdx].pt;
    cv::Point2f kpR_prev = previousStereoFeature.rightKeypoints[m.queryIdx].pt;
    cv::Point2f kpL_curr = newFeature.leftKeypoints[m.trainIdx].pt;
    cv::Point2f kpR_curr = newFeature.rightKeypoints[m.trainIdx].pt;
    // Compute disparity and check validity
    float disp_prev = kpL_prev.x - kpR_prev.x;
    float disp_curr = kpL_curr.x - kpR_curr.x;
    if (disp_prev > 1.0 && disp_curr > 1.0) {
      // Triangulate previous 3D point
      float Z_prev = K.at<double>(0, 0) * this->stereoBaseline / disp_prev;
      float X_prev = (kpL_prev.x - K.at<double>(0, 2)) * Z_prev / K.at<double>(0, 0);
      float Y_prev = (kpL_prev.y - K.at<double>(1, 2)) * Z_prev / K.at<double>(1, 1);
      pts3d_prev.emplace_back(X_prev, Y_prev, Z_prev);
      // Triangulate current 3D point (for PnP, we only need 2D in current frame)
      pts2d_curr.emplace_back(kpL_curr);
    }
  }
  if (pts3d_prev.size() < 5) {
    edge.relativePose = cv::Mat::eye(4, 4, CV_64F);
    return edge;
  }

  // Solve PnP: to estimate the rotation and translation between the previous and current frames using the 3D-2D correspondences.
  cv::Mat rvec, tvec, inliers;
  cv::solvePnPRansac(
    pts3d_prev, pts2d_curr, K,
    cv::Mat(), rvec, tvec,
    false, 100, 8.0, 0.99, inliers
  );

  // Convert rvec to rotation matrix
  cv::Mat R;
  cv::Rodrigues(rvec, R);

  // Build 4x4 transformation matrix
  edge.relativePose = cv::Mat::eye(4, 4, CV_64F);
  R.copyTo(edge.relativePose(cv::Rect(0, 0, 3, 3)));
  tvec.copyTo(edge.relativePose(cv::Rect(3, 0, 1, 3)));
  return edge;
}

void CameraManager::UpdateCameraPoseVisualization() {
  // Lock the pose mutex since we we are accessing the current pose estimate
  std::lock_guard<std::mutex> lock(this->poseMutex);

  if (this->currentPose.empty() || this->currentPose.rows != 4 || this->currentPose.cols != 4) {
    RCLCPP_WARN(this->get_logger(), "[VisulizeTrajectory] Invalid or empty pose matrix, skipping visualization.");
    return;
  }

  // Create a PoseStamped msg to visualize the camera pose trajectory
  PoseStamped poseStamped;
  poseStamped.header.stamp = this->now();
  poseStamped.header.frame_id = "camera_link";

  // Extract translation from 4x4 pose matrix
  poseStamped.pose.position.x = this->currentPose.at<double>(0, 3);
  poseStamped.pose.position.y = this->currentPose.at<double>(1, 3);
  poseStamped.pose.position.z = this->currentPose.at<double>(2, 3);

  // TODO: Use Eigen to do this conversion more robustly (cv::cv2eigen function)
  // Convert rotation matrix to quaternion
  cv::Mat R = this->currentPose(cv::Rect(0, 0, 3, 3));
  cv::Mat rvec;
  cv::Rodrigues(R, rvec);
  tf2::Quaternion q;
  const double xRot = rvec.at<double>(0);
  const double yRot = rvec.at<double>(1);
  const double zRot = rvec.at<double>(2);
  q.setRPY(zRot, yRot, xRot); // Note: tf2 uses z-y-x order for RPY
  q.normalize();
  poseStamped.pose.orientation.x = q.x();
  poseStamped.pose.orientation.y = q.y();
  poseStamped.pose.orientation.z = q.z();
  poseStamped.pose.orientation.w = q.w();

  // Add to path and publish
  this->cameraPoseEstimatePath.poses.push_back(poseStamped);
  this->cameraPoseEstimatePath.header.stamp = this->now();
  this->cameraEstimatePathPub->publish(cameraPoseEstimatePath);

  // Broadcast the camera pose as a TF transform
  geometry_msgs::msg::TransformStamped transformStamped;
  transformStamped.header.stamp = this->now();
  transformStamped.header.frame_id = "world";
  transformStamped.child_frame_id = "camera_link";
  transformStamped.transform.translation.x = poseStamped.pose.position.x;
  transformStamped.transform.translation.y = poseStamped.pose.position.y;
  transformStamped.transform.translation.z = poseStamped.pose.position.z;
  transformStamped.transform.rotation.x = poseStamped.pose.orientation.x;
  transformStamped.transform.rotation.y = poseStamped.pose.orientation.y;
  transformStamped.transform.rotation.z = poseStamped.pose.orientation.z;
  transformStamped.transform.rotation.w = poseStamped.pose.orientation.w;
  this->tfBroadcaster->sendTransform(transformStamped);
}

void CameraManager::initializePoseGraph() {
  // TODO: Initialize a graph using g20/GTSAM/Ceres
  // Nodes are camera poses, edges are relative transformations between them.
}

void CameraManager::addEdge(const Edge& edge) {
  // TODO: Add the edge to the pose graph
}

Edge CameraManager::LoopClosureDetector() {
  // TODO: Implement loop closure detection logic.
  Edge edge;
  return edge;
}

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CameraManager>());
  rclcpp::shutdown();
  return 0;
}