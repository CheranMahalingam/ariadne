#include "vslam/tracking/feature_matcher.hpp"
#include "DBoW3/FeatureVector.h"

#include <algorithm>
#include <array>
#include <cinttypes>

namespace vslam
{

FeatureMatcher::FeatureMatcher(float nn_dist_ratio)
: nn_dist_ratio_(nn_dist_ratio)
{
}

int FeatureMatcher::BoWSearch(
  const KeyFrame & key_frame, const Frame & curr_frame,
  std::vector<MapPoint> & matching_points)
{
  std::array<std::vector<int>, HISTOGRAM_BINS> rotation_histogram;
  auto curr_feature_map = curr_frame.GetFeatures();
  auto kf_feature_map = key_frame.GetFeatures();

  auto f_it = curr_feature_map.begin();
  auto kf_it = kf_feature_map.begin();
  while (f_it != curr_feature_map.end() && kf_it != kf_feature_map.end()) {
    const auto [curr_id, curr_features] = *f_it;
    const auto [kf_id, kf_features] = *kf_it;
    if (curr_id == kf_id) {
      findMatch(
        key_frame, curr_frame, kf_features, curr_features, rotation_histogram);

      f_it++;
      kf_it++;
    } else if (curr_id < kf_id) {
      f_it = curr_feature_map.lower_bound(kf_id);
    } else {
      kf_it = kf_feature_map.lower_bound(curr_id);
    }
  }

  std::array<std::pair<int, int>, HISTOGRAM_BINS> bin_sizes;
  for (int i = 0; i < HISTOGRAM_BINS; i++) {
    bin_sizes[i] = std::make_pair(int(rotation_histogram[i].size()), i);
  }
  std::ranges::sort(bin_sizes, std::ranges::greater());

  int matches = 0;
  for (int i = 0; i < FILTERED_ORIENTATION_BINS; i++) {
    auto bin = bin_sizes[i].second;
    for (auto point_idx:rotation_histogram[bin]) {
      matches++;
    }
  }

  return matches;
}

int FeatureMatcher::ProjectionSearch(
  const Frame & curr_frame, std::vector<MapPoint> & matching_points)
{
  return 0;
}

void FeatureMatcher::findMatch(
  const KeyFrame & key_frame, const Frame & curr_frame,
  const std::vector<unsigned int> & kf_descriptors,
  const std::vector<unsigned int> & curr_descriptors,
  std::array<std::vector<int>, HISTOGRAM_BINS> & rotation_histogram)
{
  for (auto kf_idx:kf_descriptors) {
    auto kf_desc = key_frame.GetDescriptor(kf_idx);

    int min_distance_idx = -1;
    int min_distance_1 = 256;
    int min_distance_2 = 256;
    for (auto curr_idx:curr_descriptors) {
      auto curr_desc = curr_frame.GetDescriptor(curr_idx);
      auto dist = hammingDistance(kf_desc, curr_desc);
      if (dist < min_distance_1) {
        min_distance_idx = curr_idx;
        min_distance_2 = min_distance_1;
        min_distance_1 = dist;
      } else if (dist < min_distance_2) {
        min_distance_2 = dist;
      }
    }

    if (min_distance_1 <= MIN_DISTANCE_THRESHOLD &&
      min_distance_1 * nn_dist_ratio_ > min_distance_2)
    {
      auto kf_point = key_frame.GetPoint(min_distance_idx);
      auto curr_point = curr_frame.GetPoint(min_distance_idx);
      float rotation = kf_point.angle - curr_point.angle;
      if (rotation < 0.0) {
        rotation += 360.0;
      }
      int bin = int((rotation - 1) * HISTOGRAM_BINS / 360.0);
      rotation_histogram[bin].push_back(min_distance_idx);
    }
  }
}

int FeatureMatcher::hammingDistance(const cv::Mat & a, const cv::Mat & b)
{
  const int * a_ptr = a.ptr<int>();
  const int * b_ptr = b.ptr<int>();
  // Compute xor such that all 1s in output denote difference in bits.
  unsigned int v = (*a_ptr) ^ (*b_ptr);
  // https://graphics.stanford.edu/~seander/bithacks.html#CountBitsSetParallel
  v = v - ((v >> 1) & 0x55555555);                    // reuse input as temporary
  v = (v & 0x33333333) + ((v >> 2) & 0x33333333);     // temp
  int dist = (((v + (v >> 4)) & 0xF0F0F0F) * 0x1010101) >> 24; // count
  return dist;
}

}  // vslam
