#ifndef VSLAM__FEATURE_MATCHER_HPP_
#define VSLAM__FEATURE_MATCHER_HPP_

#include "vslam/tracking/frame.hpp"
#include "vslam/tracking/key_frame.hpp"

#include <opencv2/core.hpp>

#include <memory>
#include <unordered_map>
#include <vector>

namespace vslam
{

class MapPoint;

class FeatureMatcher
{
public:
  static constexpr int HISTOGRAM_BINS = 30;
  static constexpr int FILTERED_ORIENTATION_BINS = 5;
  static constexpr int MIN_DISTANCE_THRESHOLD = 50;

  FeatureMatcher(float nn_dist_ratio);

  int BoWSearch(
    const KeyFrame & key_frame, const Frame & curr_frame,
    std::vector<std::shared_ptr<MapPoint>> & matching_points) const;

  int ProjectionSearch(
    Frame & curr_frame, const Frame & prev_frame,
    int search_radius_threshold) const;

private:
  void findMatchBoW(
    const KeyFrame & key_frame, const Frame & curr_frame,
    const std::vector<unsigned int> & kf_descriptors,
    const std::vector<unsigned int> & curr_descriptors,
    std::unordered_map<int, int> & visited_matches,
    std::array<std::vector<int>, HISTOGRAM_BINS> & rotation_histogram) const;

  int applyHistogramFilter(
    const std::array<std::vector<int>, HISTOGRAM_BINS> & rotation_histogram,
    const std::unordered_map<int, int> & visited_matches,
    const std::vector<std::shared_ptr<MapPoint>> & prev_map_points,
    std::vector<std::shared_ptr<MapPoint>> & curr_map_points) const;

  int hammingDistance(const cv::Mat & a, const cv::Mat & b) const;

  float nn_dist_ratio_;
};

}  // vslam

#endif  // VSLAM__FEATURE_MATCHER_HPP_
