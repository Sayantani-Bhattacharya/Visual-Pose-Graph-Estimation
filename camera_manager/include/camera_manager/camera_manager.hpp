#ifndef CAMERA_MANAGER_HPP
#define CAMERA_MANAGER_HPP
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/header.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <nav_msgs/msg/path.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>

#include <mutex>
#include <queue>
#include <vector>

using Header = std_msgs::msg::Header;
using Image = sensor_msgs::msg::Image;
using CameraInfo = sensor_msgs::msg::CameraInfo;

struct CameraIntrinsics {
  cv::Mat K; // Camera intrinsic matrix: focal length, principal point..
  cv::Mat D; // Camera distortion coefficients
};

struct Frame {
  int frameID;
  rclcpp::Time stamp;
  cv::Mat image;
  std::vector<cv::KeyPoint> keypoints;
  cv::Mat descriptors; // SIFT/ORB descriptors
  cv::Mat K; // Camera intrinsic matrix
  cv::Mat D; // Camera distortion coefficients
};

struct Edge {
  int fromID;
  int toID;
  cv::Mat relativePose; // 4x4 SE(3) Transformation matrix T_from_to
  cv::Mat covariance; // Covariance of the relative pose
  bool isLoopClosure; // Flag for loop closure edge
};

struct Feature {
  int frameID;
  std::vector<cv::KeyPoint> keypoints;
  cv::Mat descriptors; // SIFT/ORB descriptors
};

struct StereoFeature {
  int frameID;
  std::vector<cv::KeyPoint> leftKeypoints;
  std::vector<cv::KeyPoint> rightKeypoints;
  cv::Mat leftDescriptors; // SIFT/ORB descriptors for left image
  cv::Mat rightDescriptors; // SIFT/ORB descriptors for right image
};

class CameraManager : public rclcpp::Node {
public:
  CameraManager();
  ~CameraManager() = default;

  Feature FeatureExtractor(const Frame& frame);
  StereoFeature StereoFeatureExtractor(const Frame& leftFrame, const Frame& rightFrame);
  Edge MonocularCameraPoseEstimation(const Feature& feature);
  Edge StereoCameraPoseEstimation(const StereoFeature& feature);
  Edge LoopClosureDetector();
  void GraphBuilder(const Edge& estimatedPose, const Edge& loopConstraints);
  void VisualizeGraph();
  void VisulizeTrajectory(Edge odomEdge);

private:
  // Timer for camera image processing
  rclcpp::TimerBase::SharedPtr timer;


  rclcpp::Subscription<Image>::SharedPtr cameraSub;
  rclcpp::Subscription<Image>::SharedPtr leftCameraSub;
  rclcpp::Subscription<Image>::SharedPtr rightCameraSub;
  rclcpp::Subscription<CameraInfo>::SharedPtr cameraInfoSub;
  rclcpp::Subscription<CameraInfo>::SharedPtr leftCameraInfoSub;
  rclcpp::Subscription<CameraInfo>::SharedPtr rightCameraInfoSub;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pathPub;
  nav_msgs::msg::Path pathMsg;



  void timerCallback();


  // Timer Frequency
  float timerFreq; // [Hz]
  // Camera Intrinsics Info
  CameraIntrinsics cameraIntrinsics;
  bool collectedCameraInfo = false;
  CameraIntrinsics leftCameraIntrinsics;
  bool collectedLeftCameraInfo = false;
  CameraIntrinsics rightCameraIntrinsics;
  bool collectedRightCameraInfo = false;


  // Mutex for safely accessing the frame queue
  std::mutex frameMutex;
  std::mutex leftFrameMutex;
  std::mutex rightFrameMutex;

  // Container for frames
  std::queue<Frame> frameQueue;
  std::queue<Frame> leftFrameQueue;
  std::queue<Frame> rightFrameQueue;

  // Container for features
  std::map <int, Feature> featureMap;
  std::map <int, StereoFeature> stereoFeatureMap;
  // Container for edges
  std::vector<Edge> allEdges;

};
#endif // !CAMERA_MANAGER_HPP