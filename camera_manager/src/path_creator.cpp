#include "path_creator.hpp"

PathCreator::PathCreator() : Node("path_creator"), tfBuffer(this->get_clock()), tfListener(tfBuffer) {
  RCLCPP_INFO(this->get_logger(), "Path Creator Node Initialized");

  // Declare parameters.
  this->timerFreq = this->declare_parameter("timer_frequency", 10.0); // [Hz] Timer for managing pose-graph
  // Get params from config file.
  this->timerFreq = this->get_parameter("timer_frequency").as_double();

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
        if (mCurrentState == State::RECORDING) {
          this->cameraPath = *msg; // Store the received camera path
        } 
        else if(mCurrentState == State::RACING) {
          this->robotPath = *msg; // Store the received camera path
        }
        else {
          this->robotPath = *msg; // Store the received camera path as robot path
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

  this->startRaceService = this->create_service<std_srvs::srv::Trigger>(
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