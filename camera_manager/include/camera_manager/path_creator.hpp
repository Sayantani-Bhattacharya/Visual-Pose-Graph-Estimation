#ifndef PATH_CREATOR_HPP
#define PATH_CREATOR_HPP
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <nav_msgs/msg/path.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include "tf2_ros/transform_broadcaster.h"
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <mutex>


class PathCreator : public rclcpp::Node {
public:
  PathCreator();
  ~PathCreator() = default;

  nav_msgs::msg::Path getCameraPath() {
    std::lock_guard<std::mutex> lock(cameraPathMutex); // Lock the mutex for thread safety
    return cameraPath; // Return the camera path
  }

  float score(const nav_msgs::msg::Path& reference, const nav_msgs::msg::Path& actual);

  // State Machine
  enum State {
    IDLE,
    RECORDING,
    END_RECORDING,
    RESETTING,
    RACING,
    END_RACING
  };

  nav_msgs::msg::Path extractPath(const rclcpp::Time& start, const rclcpp::Time& end);

private:
  rclcpp::TimerBase::SharedPtr timer; // Timer for managing path updates
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr scorePub; // Publisher for the score text
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr robotPathPub; // Publisher for the robot path
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr racePathPub; // Publisher for the camera path
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr cameraPathSub; // Subscription to receive camera path
  tf2_ros::Buffer tfBuffer; // Buffer for storing transforms
  tf2_ros::TransformListener tfListener; // Listener for transforms

  // Service calls
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr startRecordingService; // Service to start recording
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stopRecordingService; // Service to stop recording
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr resetPathService; // Service to reset the path
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr startRaceService; // Service to start the race.
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr endRaceService; // Service to end the race.

  float timerFreq; // Timer frequency [Hz]
  PathCreator::State mCurrentState = IDLE; // Current state of the path creator
  std::mutex cameraPathMutex; // Mutex for thread safety
  nav_msgs::msg::Path cameraPath; // Path to store camera pose estimates
  nav_msgs::msg::Path racePath; // Path to store camera pose estimates during racing
  nav_msgs::msg::Path robotPath; // Path to store robot pose estimates

  rclcpp::Time mRecordStartTimestamp; // Timestamp when recording started
  rclcpp::Time mRecordEndTimestamp; // Timestamp when recording started
  rclcpp::Time mStartRaceTimestamp; // Timestamp when racing started
  rclcpp::Time mEndRaceTimestamp; // Timestamp when racing ended


  // Trigger calls.
  bool startRecord = false; // Flag to indicate if recording has started
  bool startRace = false; // Flag to indicate if racing has started
  visualization_msgs::msg::Marker createScoreMarker(float score, const std::string& frame_id);

  void timerCallback();

};
#endif // !PATH_CREATOR_HPP