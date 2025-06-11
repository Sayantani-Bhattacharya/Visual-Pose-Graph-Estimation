#ifndef PATH_CREATOR_HPP
#define PATH_CREATOR_HPP
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <nav_msgs/msg/path.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include "tf2_ros/transform_broadcaster.h"
#include <geometry_msgs/msg/pose_stamped.hpp>


class PathCreator : public rclcpp::Node {
public:
  PathCreator();
  ~PathCreator() = default;

  nav_msgs::msg::Path getCameraPath() {
    std::lock_guard<std::mutex> lock(cameraPathMutex); // Lock the mutex for thread safety
    return cameraPath; // Return the camera path
  }

  float score(const nav_msgs::msg::Path& reference, const nav_msgs::msg::Path& actual);

private:
  rclcpp::TimerBase::SharedPtr timer; // Timer for managing path updates
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr scorePub; // Publisher for the score text
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr robotPathPub; // Publisher for the robot path
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr cameraPathSub; // Subscription to receive camera path
  tf2_ros::Buffer tfBuffer; // Buffer for storing transforms
  tf2_ros::TransformListener tfListener; // Listener for transforms

  float timerFreq; // Timer frequency [Hz]
  bool recordingPath; // Flag to indicate if robotPath is being recorded or not
  std::mutex cameraPathMutex; // Mutex for thread safety
  nav_msgs::msg::Path cameraPath; // Path to store camera pose estimates
  nav_msgs::msg::Path robotPath; // Path to store robot pose estimates

  visualization_msgs::msg::Marker createScoreMarker(float score, const std::string& frame_id);

  void timerCallback();

};
#endif // !PATH_CREATOR_HPP