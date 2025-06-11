#include "path_creator.hpp"

PathCreator::PathCreator() : Node("path_creator"), tfBuffer(this->get_clock()), tfListener(tfBuffer) {
  RCLCPP_INFO(this->get_logger(), "Path Creator Node Initialized");

  // Declare parameters.
  this->timerFreq = this->declare_parameter("timer_frequency", 10.0); // [Hz] Timer for managing pose-graph
  // Get params from config file.
  this->timerFreq = this->get_parameter("timer_frequency").as_double();

  // Setup score publisher
  this->scorePub = this->create_publisher<visualization_msgs::msg::Marker>("/path_creator/path_score", 10);

  // Setup Path publisher for robot trajectory
  this->robotPathPub = this->create_publisher<nav_msgs::msg::Path>("robot_trajectory", 10);
  this->racePathPub = this->create_publisher<nav_msgs::msg::Path>("race_trajectory", 10);

  this->robotPath.header.frame_id = "odom"; // Set the frame ID for the robot path
  this->robotPath.header.stamp = this->now(); // Initialize the timestamp
  this->robotPath.poses.clear(); // Clear any existing poses
  // Setup Subscription to receive camera path
  this->cameraPathSub = this->create_subscription<nav_msgs::msg::Path>(
    "/zed/zed_node/path_odom", 10,
    [this](const nav_msgs::msg::Path::SharedPtr msg) {
      {
        std::lock_guard<std::mutex> lock(this->cameraPathMutex); // Lock the mutex for thread safety
        if (mCurrentState == State::RECORDING) {
          this->robotPath = *msg; // Store the received camera path
        } else if (mCurrentState == State::RACING) {
          this->racePath = *msg; // Store the received camera path
        } else {
          this->cameraPath = *msg; // Store the received camera path as robot path
        }
      }
  }
  );

  // Setup Service calls
  this->startRecordingService = this->create_service<std_srvs::srv::Trigger>(
    "start_recording", [this](const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
    mCurrentState = State::RECORDING; // Set the flag to start recording
    response->success = true;
    response->message = "[Server Call] Started recording path.";
  }
  );

  this->stopRecordingService = this->create_service<std_srvs::srv::Trigger>(
    "stop_recording", [this](const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
    mCurrentState = State::END_RECORDING; // Set the flag to stop recording
    response->success = true;
    response->message = "[Server Call] Stopped recording path.";
  }
  );

  this->resetPathService = this->create_service<std_srvs::srv::Trigger>(
    "reset_path", [this](const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
    mCurrentState = State::RESETTING; // Set the flag to reset the path
    this->cameraPath.poses.clear(); // Clear the camera path
    this->robotPath.poses.clear(); // Clear the robot path
    response->success = true;
    response->message = "[Server Call] Path reset.";
  }
  );

  this->startRaceService = this->create_service<std_srvs::srv::Trigger>(
    "start_race", [this](const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
    mCurrentState = State::RACING; // Start race
    response->success = true;
    response->message = "[Server Call] The Race Begins.";
  }
  );

  this->endRaceService = this->create_service<std_srvs::srv::Trigger>(
    "end_race", [this](const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
    mCurrentState = State::END_RACING; // End race
    response->success = true;
    response->message = "[Server Call] The Race Ended.";
  }
  );

  this->timer = this->create_wall_timer(
    std::chrono::milliseconds(static_cast<int>(1000 / this->timerFreq)),
    std::bind(&PathCreator::timerCallback, this)
  );
}

float PathCreator::score(const nav_msgs::msg::Path& reference, const nav_msgs::msg::Path& actual) {
  // Compare how closely the path follows the reference path
  const int N = std::min(reference.poses.size(), actual.poses.size());
  float totalError = 0.0f;
  float maxDistance = 0.0f;
  for (int i = 0; i < N; ++i) {
    const auto& refPose = reference.poses[i].pose;
    const auto& actPose = actual.poses[i].pose;

    // Calculate the Euclidean distance between the reference and actual poses
    float dx = refPose.position.x - actPose.position.x;
    float dy = refPose.position.y - actPose.position.y;
    float dz = refPose.position.z - actPose.position.z;
    float distance = std::sqrt(dx * dx + dy * dy + dz * dz);
    maxDistance = std::max(maxDistance, distance); // Track the maximum distance for normalization
    totalError += distance; // Accumulate the total error
  }
  totalError /= N; // Average error over all poses
  // Normalize the score between 0 and 1 since score should be a percentage
  float score = std::max(0.0f, 1.0f - (totalError / maxDistance)); // Return a score between 0 and 1
  this->scorePub->publish(createScoreMarker(score, reference.header.frame_id)); // Publish the score marker
  return score;
}

visualization_msgs::msg::Marker PathCreator::createScoreMarker(float score, const std::string& frame_id) {
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = frame_id;
  marker.header.stamp = this->now();
  marker.ns = "path_score";
  marker.id = 0;
  marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.pose.position.x = 0.0;
  marker.pose.position.y = 0.0;
  marker.pose.position.z = 1.0; // Position above the ground
  marker.pose.orientation.w = 1.0; // No rotation
  const float textSize = 0.5f; // Size of the text
  marker.scale.x = textSize;
  marker.scale.y = textSize;
  marker.scale.z = textSize;
  marker.color.r = 1.0; // White color
  marker.color.g = 1.0;
  marker.color.b = 1.0;
  marker.color.a = 1.0; // Fully opaque
  marker.text = "Path Score: " + std::to_string(score * 100) + "%"; // Display score as percentage
  return marker;
}

void PathCreator::timerCallback() {

  // Publish robot path and race path all the time if not empty.
  if (!this->robotPath.poses.empty()) {
    this->robotPath.header.stamp = this->now(); // Update the timestamp
    this->robotPathPub->publish(this->robotPath); // Publish the robot path
  }
  if (!this->racePath.poses.empty()) {
    this->racePath.header.stamp = this->now(); // Update the timestamp
    this->racePathPub->publish(this->racePath); // Publish the robot path
  }
  if (this->mCurrentState == State::RACING) {
    this->score(this->robotPath, this->racePath);
  }
}

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PathCreator>());
  rclcpp::shutdown();
  return 0;
}