#ifndef VSLAM__FEATURE_MATCHER_HPP_
#define VSLAM__FEATURE_MATCHER_HPP_

#include "vslam/mapping/map_point.hpp"
#include "vslam/tracking/frame.hpp"
#include "vslam/tracking/key_frame.hpp"

#include <opencv2/core.hpp>

#include <vector>

namespace vslam
{

class FeatureMatcher
{
public:
  static constexpr int HISTOGRAM_BINS = 30;
  static constexpr int FILTERED_ORIENTATION_BINS = 5;
  static constexpr int MIN_DISTANCE_THRESHOLD = 50;

  FeatureMatcher(float nn_dist_ratio);

  int BoWSearch(
    const KeyFrame & key_frame, const Frame & curr_frame,
    std::vector<MapPoint> & matching_points);

  int ProjectionSearch(
    const Frame & curr_frame, std::vector<MapPoint> & matching_points);

private:
  void findMatch(
    const KeyFrame & key_frame, const Frame & curr_frame,
    const std::vector<unsigned int> & kf_descriptors,
    const std::vector<unsigned int> & curr_descriptors,
    std::array<std::vector<int>, HISTOGRAM_BINS> & rotation_histogram);

  int hammingDistance(const cv::Mat & a, const cv::Mat & b);

  float nn_dist_ratio_;
};

}  // vslam

#endif  // VSLAM__FEATURE_MATCHER_HPP_
