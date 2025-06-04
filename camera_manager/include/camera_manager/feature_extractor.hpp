#ifndef FEATURE_EXTRACTOR_HPP
#define FEATURE_EXTRACTOR_HPP
#include <rclcpp/rclcpp.hpp>
#include <opencv2/opencv.hpp>

struct CameraIntrinsics {
  cv::Mat K; // Camera intrinsic matrix: focal length, principal point..
  cv::Mat D; // Camera distortion coefficients
};

struct Frame {
  int frameID;
  rclcpp::Time stamp;
  cv::Mat image;
  std::vector<cv::KeyPoint> keypoints;
  CameraIntrinsics intrinsics; // Camera intrinsics
  cv::Mat descriptors; // SIFT/ORB descriptors
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

class FeatureExtractor {
public:
  FeatureExtractor() = default;
  ~FeatureExtractor() = default;
};

#endif // !FEATURE_EXTRACTOR_HPP