#ifndef CAMERA_MANAGER_HPP
#define CAMERA_MANAGER_HPP
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/header.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.hpp>

using Header = std_msgs::msg::Header;
using Image = sensor_msgs::msg::Image;

struct OpenCVImage {
  Header header;
  int frameID;
  cv::Mat image;
  std::string encoding;
};

struct FeatureMap {
  int frameID;
  std::vector<cv::KeyPoint> keypoints;
  cv::Mat descriptors;
};

struct PoseConstraints {
  int frameID1;
  int frameID2;
  cv::Mat relativePose;
};

class CameraManager : public rclcpp::Node {
public:
  CameraManager();
  ~CameraManager() = default;

  FeatureMap FeatureExtractor(const OpenCVImage& frame);
  PoseConstraints CameraPoseEstimation(const FeatureMap& featureMap);
  PoseConstraints LoopClosureDetector();
  void GraphBuilder(const PoseConstraints& estimatedPose, const PoseConstraints& loopConstraints);
  void VisualizeGraph();

private:
  // Timer for camera image processing
  rclcpp::TimerBase::SharedPtr timer;
  // Subscriber to camera image
  rclcpp::Subscription<Image>::SharedPtr cameraSub;

  void timerCallback();

  // Timer Frequency
  float timerFreq; // [Hz]

  // OpenCV Image instance of camera image
  OpenCVImage cvImage;

  // Mutex for safely accessing the cvImage in the timer
  std::mutex cvImageMutex;

  // Feature Map
  std::map<int, std::pair<std::vector<cv::KeyPoint>, cv::Mat>> featureMap;

};
#endif // !CAMERA_MANAGER_HPP