#include "path_creator.hpp"

PathCreator::PathCreator() : Node("path_creator"), tfBuffer(this->get_clock()), tfListener(tfBuffer) {
  RCLCPP_INFO(this->get_logger(), "Path Creator Node Initialized");

  // Declare parameters.
  this->timerFreq = this->declare_parameter("timer_frequency", 10.0); // [Hz] Timer for managing pose-graph
  // Get params from config file.
  this->timerFreq = this->get_parameter("timer_frequency").as_double();

  // Setup score publisher
  this->scorePub = this->create_publisher<std_msgs::msg::Float32>("/path_creator/path_score", 10);

  // Setup Path publisher for robot trajectory
  this->robotPathPub = this->create_publisher<nav_msgs::msg::Path>("robot_trajectory", 10);
  this->robotPath.header.frame_id = "odom"; // Set the frame ID for the robot path
  this->robotPath.header.stamp = this->now(); // Initialize the timestamp
  this->robotPath.poses.clear(); // Clear any existing poses
  // Setup Subscription to receive camera path
  this->cameraPathSub = this->create_subscription<nav_msgs::msg::Path>(
    "/zed/zed_node/path_odom", 10,
    [this](const nav_msgs::msg::Path::SharedPtr msg) {
      {
        std::lock_guard<std::mutex> lock(this->cameraPathMutex); // Lock the mutex for thread safety
        if (recordingPath) {
          this->cameraPath = *msg; // Store the received camera path
        } else {
          this->robotPath = *msg; // Store the received camera path as robot path
        }
      }
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
  if (N == 0) {
    RCLCPP_WARN(this->get_logger(), "No poses in either path to score.");
    return 0.0f; // No poses to compare
  }
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
  return std::max(0.0f, 1.0f - (totalError / maxDistance)); // Return a score between 0 and 1
}

void PathCreator::timerCallback() {
  // Listen for the camera TF while "recording"
  // And that builds the robotPath (different than cameraPath)
}

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PathCreator>());
  rclcpp::shutdown();
  return 0;
}