#include "vslam/mapping/map_point.hpp"
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
  std::vector<std::shared_ptr<MapPoint>> & matching_points) const
{
  std::array<std::vector<int>, HISTOGRAM_BINS> rotation_histogram;
  const auto & kf_map_points = key_frame.GetMapPoints();
  const auto & curr_feature_map = curr_frame.feat_vector;
  const auto & kf_feature_map = key_frame.feat_vector;

  auto f_it = curr_feature_map.begin();
  auto kf_it = kf_feature_map.begin();
  while (f_it != curr_feature_map.end() && kf_it != kf_feature_map.end()) {
    const auto [curr_id, curr_features] = *f_it;
    const auto [kf_id, kf_features] = *kf_it;
    if (curr_id == kf_id) {
      for (auto kf_idx:kf_features) {
        if (kf_map_points[kf_idx] == nullptr || kf_map_points[kf_idx]->Culled()) {
          continue;
        }

        auto kf_desc = key_frame.descriptors[kf_idx];

        int min_distance_idx = -1;
        int min_distance_1 = 256;
        int min_distance_2 = 256;
        for (auto curr_idx:curr_features) {
          if (matching_points[curr_idx] != nullptr) {
            continue;
          }

          auto curr_desc = curr_frame.descriptors[curr_idx];
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
          matching_points[min_distance_idx] = kf_map_points[kf_idx];
          float rotation = key_frame.key_points[kf_idx].angle - \
            curr_frame.key_points[min_distance_idx].angle;
          if (rotation < 0.0) {
            rotation += 360.0;
          }
          int bin = int((rotation - 1) * HISTOGRAM_BINS / 360.0);
          rotation_histogram[bin].push_back(min_distance_idx);
        }
      }

      f_it++;
      kf_it++;
    } else if (curr_id < kf_id) {
      f_it = curr_feature_map.lower_bound(kf_id);
    } else {
      kf_it = kf_feature_map.lower_bound(curr_id);
    }
  }

  return applyHistogramFilter(rotation_histogram, matching_points);
}

int FeatureMatcher::ProjectionSearch(
  Frame & curr_frame, const Frame & prev_frame,
  int search_radius_threshold) const
{
  std::array<std::vector<int>, HISTOGRAM_BINS> rotation_histogram;
  cv::Mat curr_rotation = curr_frame.camera_world_transform.rowRange(0, 3).colRange(0, 3);
  cv::Mat curr_translation = curr_frame.camera_world_transform.rowRange(0, 3).col(3);
  // cv::Mat prev_rotation = prev_frame.camera_world_transform.rowRange(0, 3).colRange(0, 3);
  // cv::Mat prev_translation = prev_frame.camera_world_transform.rowRange(0, 3).col(3);

  for (int i = 0; i < int(prev_frame.key_points.size()); i++) {
    auto map_point = prev_frame.map_points[i];
    if (map_point == nullptr || map_point->Culled()) {
      continue;
    }

    cv::Mat world_pos = map_point->GetWorldPos();
    cv::Mat camera_pos = curr_rotation * world_pos + curr_translation;
    float camera_x = camera_pos.at<float>(0);
    float camera_y = camera_pos.at<float>(1);
    float camera_z = camera_pos.at<float>(2);
    if (camera_z < 0) {
      continue;
    }

    float u = curr_frame.camera_params.fx * camera_x / camera_z + \
      curr_frame.camera_params.cx;
    float v = curr_frame.camera_params.fy * camera_y / camera_z + \
      curr_frame.camera_params.cy;
    int prev_level = prev_frame.key_points[i].octave;
    // Adjust search radius according to feature scale to account for sparsity
    // of features.
    float radius = search_radius_threshold * prev_frame.image_scale_factors[prev_level];
    // TODO: Reduce number of levels for which matching features are searched
    // for using the direction of motion from the previous frame.
    //
    // int prev_octave = prev_frame.key_points[i].octave;
    auto neighbour_features = curr_frame.FindNeighbourFeatures(u, v, radius);
    if (neighbour_features.empty()) {
      continue;
    }

    cv::Mat prev_descriptor = map_point->GetDescriptor();
    int min_distance = 256;
    int min_distance_idx = -1;
    for (auto feature_idx:neighbour_features) {
      if (curr_frame.map_points[feature_idx] != nullptr) {
        continue;
      }

      if (curr_frame.stereo_key_points[feature_idx] > 0) {
        float bf = curr_frame.camera_params.fx * \
          curr_frame.camera_params.depth_baseline / 1000.0;
        float u_stereo = u - bf / camera_z;
        if (std::abs(u_stereo - curr_frame.depth_points[feature_idx]) > radius) {
          continue;
        }
      }

      cv::Mat curr_descriptor = curr_frame.descriptors[feature_idx];
      auto dist = hammingDistance(prev_descriptor, curr_descriptor);
      if (dist < min_distance) {
        min_distance = dist;
        min_distance_idx = feature_idx;
      }
    }

    if (min_distance <= MIN_DISTANCE_THRESHOLD) {
      curr_frame.map_points[min_distance_idx] = map_point;
      float rotation = prev_frame.key_points[i].angle - \
        curr_frame.key_points[min_distance_idx].angle;
      if (rotation < 0.0) {
        rotation += 360.0;
      }
      int bin = int((rotation - 1) * HISTOGRAM_BINS / 360.0);
      rotation_histogram[bin].push_back(min_distance_idx);
    }
  }

  return applyHistogramFilter(rotation_histogram, curr_frame.map_points);
}

int FeatureMatcher::applyHistogramFilter(
  const std::array<std::vector<int>, HISTOGRAM_BINS> & rotation_histogram,
  std::vector<std::shared_ptr<MapPoint>> & curr_map_points) const
{
  std::array<std::pair<int, int>, HISTOGRAM_BINS> bin_sizes;
  for (int i = 0; i < HISTOGRAM_BINS; i++) {
    bin_sizes[i] = std::make_pair(int(rotation_histogram[i].size()), i);
  }
  std::ranges::sort(bin_sizes, std::ranges::greater());

  int matches = bin_sizes[0].first;
  for (int i = 1; i < HISTOGRAM_BINS; i++) {
    auto [size, bin] = bin_sizes[i];
    if (i < FILTERED_ORIENTATION_BINS && size > 0.1 * bin_sizes[0].first) {
      matches += size;
      continue;
    }
    for (auto point_idx:rotation_histogram[bin]) {
      curr_map_points[point_idx] = nullptr;
    }
  }
  return matches;
}

int FeatureMatcher::hammingDistance(const cv::Mat & a, const cv::Mat & b) const
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
