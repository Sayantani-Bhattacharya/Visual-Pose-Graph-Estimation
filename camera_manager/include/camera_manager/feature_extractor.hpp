#ifndef FEATURE_EXTRACTOR_HPP
#define FEATURE_EXTRACTOR_HPP
#include <opencv2/opencv.hpp>

struct Feature {
  int frameID;
  std::vector<cv::KeyPoint> keypoints;
  cv::Mat descriptors; // SIFT/ORB descriptors
};

class FeatureExtractor {
public:
  FeatureExtractor() = default;
  ~FeatureExtractor() = default;


};

#endif // !FEATURE_EXTRACTOR_HPP