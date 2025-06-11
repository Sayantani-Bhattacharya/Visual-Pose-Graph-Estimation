#include "path_creator.hpp"

PathCreator::PathCreator() : Node("path_creator"), tfBuffer(this->get_clock()), tfListener(tfBuffer) {
  RCLCPP_INFO(this->get_logger(), "Path Creator Node Initialized");

  // Declare parameters.
  this->timerFreq = this->declare_parameter("timer_frequency", 10.0); // [Hz] Timer for managing pose-graph
  // Get params from config file.
  this->timerFreq = this->get_parameter("timer_frequency").as_double();

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
        this->cameraPath = *msg; // Store the received camera path as robot path
      }
  }
  );

  // Setup Service calls
  this->startRecordingService = this->create_service<std_srvs::srv::Trigger>(
    "start_recording", [this]([[maybe_unused]] const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
    mCurrentState = State::RECORDING; // Set the flag to start recording
    startRecord = true;
    // Extract the timing 
    mRecordStartTimestamp = this->now(); // Store the current time as the start time
    response->success = true;
    response->message = "[Server Call] Started recording path.";
  }
  );

  this->stopRecordingService = this->create_service<std_srvs::srv::Trigger>(
    "stop_recording", [this]([[maybe_unused]] const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
    mCurrentState = State::END_RECORDING; // Set the flag to stop recording
    mRecordEndTimestamp = this->now(); // Store the current time as the start time
    response->success = true;
    response->message = "[Server Call] Stopped recording path.";
  }
  );

  this->resetPathService = this->create_service<std_srvs::srv::Trigger>(
    "reset_path", [this]([[maybe_unused]] const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
    mCurrentState = State::RESETTING; // Set the flag to reset the path
    this->cameraPath.poses.clear(); // Clear the camera path
    this->robotPath.poses.clear(); // Clear the robot path
    this->racePath.poses.clear();
    response->success = true;
    response->message = "[Server Call] Path reset.";
  }
  );

  this->startRaceService = this->create_service<std_srvs::srv::Trigger>(
    "start_race", [this]([[maybe_unused]] const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
    mCurrentState = State::RACING; // Start race
    startRace = true; // Set the flag to start racing
    mStartRaceTimestamp = this->now(); // Store the current time as the start time
    response->success = true;
    response->message = "[Server Call] The Race Begins.";
  }
  );

  this->endRaceService = this->create_service<std_srvs::srv::Trigger>(
    "end_race", [this]([[maybe_unused]] const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
    mCurrentState = State::END_RACING; // End race
    mEndRaceTimestamp = this->now(); // Store the current time as the end time
    response->success = true;
    response->message = "[Server Call] The Race Ended.";
  }
  );
  this->timer = this->create_wall_timer(
    std::chrono::milliseconds(static_cast<int>(1000 / this->timerFreq)),
    std::bind(&PathCreator::timerCallback, this)
  );
}

nav_msgs::msg::Path PathCreator::extractPath(const rclcpp::Time& start, const rclcpp::Time& end) {
  nav_msgs::msg::Path path;
  std::lock_guard<std::mutex> lock(cameraPathMutex);
  for (const auto& pose_stamped : cameraPath.poses) {
    const auto& stamp = pose_stamped.header.stamp;
    rclcpp::Time t(stamp.sec, stamp.nanosec, RCL_ROS_TIME);
    if (t >= start && t <= end) {
      path.poses.push_back(pose_stamped);
    } else if (t > end) {
      break; // Stop if we have passed the end time
    }
  }
  path.header.frame_id = cameraPath.header.frame_id; // Set the frame ID from the camera path
  path.header.stamp = this->now(); // Update the header timestamp
  return path;
}

void PathCreator::timerCallback() {
  // Publish robot path and race path all the time if not empty.

  if (mCurrentState == State::RECORDING) {
    this->robotPath = this->extractPath(mRecordStartTimestamp, this->now()); // Extract the path from start to now
  } else if (mCurrentState == State::END_RECORDING) {
    this->robotPath = this->extractPath(mRecordStartTimestamp, mRecordEndTimestamp); // Extract the path from start to now
  } else if (mCurrentState == State::RACING) {
    this->racePath = this->extractPath(mStartRaceTimestamp, this->now()); // Extract the path from start to now
  } else if (mCurrentState == State::END_RACING) {
    this->racePath = this->extractPath(mStartRaceTimestamp, mEndRaceTimestamp); // Extract the path from start to now
  } else if (mCurrentState == State::RESETTING) {
    mStartRaceTimestamp = this->now();
    mEndRaceTimestamp = this->now();
    mRecordStartTimestamp = this->now();
    mRecordEndTimestamp = this->now();
    this->racePath.poses.clear(); // Clear the camera path
    this->robotPath.poses.clear(); // Clear the robot path
  }

  this->robotPath.header.frame_id = "odom"; // Set the frame ID for the robot path
  this->racePath.header.frame_id = "odom"; // Set the frame ID
  this->robotPath.header.stamp = this->now(); // Update the timestamp
  this->robotPathPub->publish(this->robotPath); // Publish the robot path
  this->racePath.header.stamp = this->now(); // Update the timestamp
  this->racePathPub->publish(this->racePath); // Publish the robot path
}

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PathCreator>());
  rclcpp::shutdown();
  return 0;
}