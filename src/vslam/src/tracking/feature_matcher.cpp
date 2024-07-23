#include "vslam/mapping/map_point.hpp"
#include "vslam/tracking/feature_matcher.hpp"
#include "vslam/utils.hpp"

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
  KeyFrame & key_frame, const Frame & curr_frame,
  std::vector<std::shared_ptr<MapPoint>> & matching_points) const
{
  std::array<std::vector<int>, HISTOGRAM_BINS> rotation_histogram;
  auto valid_matches = std::vector<bool>(matching_points.size(), false);

  auto kf_map_points = key_frame.GetMapPoints();
  auto curr_feature_map = curr_frame.feat_vector;
  auto kf_feature_map = key_frame.feat_vector;
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
        int min_distance_1 = MAX_DESCRIPTOR_DISTANCE;
        int min_distance_2 = MAX_DESCRIPTOR_DISTANCE;
        for (auto curr_idx:curr_features) {
          if (matching_points[curr_idx] != nullptr) {
            continue;
          }

          auto curr_desc = curr_frame.descriptors[curr_idx];
          auto dist = hamming_distance(kf_desc, curr_desc);
          if (dist < min_distance_1) {
            min_distance_idx = curr_idx;
            min_distance_2 = min_distance_1;
            min_distance_1 = dist;
          } else if (dist < min_distance_2) {
            min_distance_2 = dist;
          }
        }

        if (min_distance_1 <= MIN_DISTANCE_THRESHOLD &&
          min_distance_1 > min_distance_2 * nn_dist_ratio_)
        {
          matching_points[min_distance_idx] = kf_map_points[kf_idx];
          valid_matches[min_distance_idx] = true;
          float rotation = (key_frame.key_points[kf_idx].angle -
            curr_frame.key_points[min_distance_idx].angle);
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

  int matches = applyHistogramFilter(rotation_histogram, valid_matches);
  for (int i = 0; i < int(matching_points.size()); i++) {
    if (!valid_matches[i]) {
      matching_points[i] = nullptr;
    }
  }
  return matches;
}

int FeatureMatcher::ProjectionSearch(
  Frame & curr_frame, const Frame & prev_frame,
  int search_radius_threshold) const
{
  std::array<std::vector<int>, HISTOGRAM_BINS> rotation_histogram;
  auto valid_matches = std::vector<bool>(curr_frame.map_points.size(), false);

  // Determine whether the system has moved forward relative to prev_frame.
  cv::Mat relative_pos = (prev_frame.pose.rotation_cw *
    curr_frame.pose.translation_wc + prev_frame.pose.translation_cw);
  auto forward = relative_pos.at<float>(2) >= 0.0;

  for (int i = 0; i < int(prev_frame.key_points.size()); i++) {
    auto map_point = prev_frame.map_points[i];
    if (map_point == nullptr || prev_frame.outliers[i]) {
      continue;
    }

    cv::Mat world_pos = map_point->GetWorldPos();
    cv::Mat camera_pos = (curr_frame.pose.rotation_cw * world_pos +
      curr_frame.pose.translation_cw);
    float camera_x = camera_pos.at<float>(0);
    float camera_y = camera_pos.at<float>(1);
    float camera_z = camera_pos.at<float>(2);
    if (camera_z < 0) {
      continue;
    }

    float u = (curr_frame.camera_params.fx * camera_x / camera_z +
      curr_frame.camera_params.cx);
    float v = (curr_frame.camera_params.fy * camera_y / camera_z +
      curr_frame.camera_params.cy);
    int prev_level = prev_frame.key_points[i].octave;
    // Adjust search radius according to feature scale to account for sparsity
    // of features.
    float radius = search_radius_threshold * prev_frame.image_scale_factors[prev_level];
    std::vector<int> neighbour_features;
    if (forward) {
      neighbour_features = curr_frame.FindNeighbourFeatures(
        u, v, radius, prev_level);
    } else {
      neighbour_features = curr_frame.FindNeighbourFeatures(
        u, v, radius, 0, prev_level);
    }
    if (neighbour_features.empty()) {
      continue;
    }

    auto prev_descriptor = map_point->GetDescriptor();
    int min_distance = MAX_DESCRIPTOR_DISTANCE;
    int min_distance_idx = -1;
    for (auto feature_idx:neighbour_features) {
      if (curr_frame.map_points[feature_idx] != nullptr &&
        curr_frame.map_points[feature_idx]->GetNumObservations() > 0)
      {
        continue;
      }

      if (curr_frame.stereo_key_points[feature_idx] > 0) {
        float bf = (curr_frame.camera_params.fx *
          curr_frame.camera_params.depth_baseline / 1000.0);
        float u_stereo = u - bf / camera_z;
        if (std::abs(u_stereo - curr_frame.depth_points[feature_idx]) > radius) {
          continue;
        }
      }

      cv::Mat curr_descriptor = curr_frame.descriptors[feature_idx];
      auto dist = hamming_distance(prev_descriptor, curr_descriptor);
      if (dist < min_distance) {
        min_distance = dist;
        min_distance_idx = feature_idx;
      }
    }

    if (min_distance <= MIN_DISTANCE_THRESHOLD) {
      curr_frame.map_points[min_distance_idx] = map_point;
      valid_matches[min_distance_idx] = true;
      float rotation = (prev_frame.key_points[i].angle -
        curr_frame.key_points[min_distance_idx].angle);
      if (rotation < 0.0) {
        rotation += 360.0;
      }
      int bin = int((rotation - 1) * HISTOGRAM_BINS / 360.0);
      rotation_histogram[bin].push_back(min_distance_idx);
    }
  }

  int matches = applyHistogramFilter(rotation_histogram, valid_matches);
  for (int i = 0; i < int(curr_frame.map_points.size()); i++) {
    if (!valid_matches[i]) {
      curr_frame.map_points[i] = nullptr;
    }
  }
  return matches;
}

int FeatureMatcher::ProjectionSearch(
  Frame & curr_frame, const std::vector<std::shared_ptr<MapPoint>> & map_points,
  int search_radius_threshold) const
{
  int matches = 0;
  for (auto mp:map_points) {
    if (mp->Culled() || !mp->tracked) {
      continue;
    }

    auto predicted_level = mp->scale_level;
    float radius = search_radius_threshold * 4.0 * curr_frame.image_scale_factors[predicted_level];
    auto neighbour_features = curr_frame.FindNeighbourFeatures(
      mp->projected_x, mp->projected_y, radius,
      predicted_level - 1, predicted_level);
    if (neighbour_features.empty()) {
      continue;
    }
    auto mp_descriptor = mp->GetDescriptor();

    int min_distance_idx = -1;
    int min_distance_level_1 = -1;
    int min_distance_level_2 = -1;
    int min_distance_1 = MAX_DESCRIPTOR_DISTANCE;
    int min_distance_2 = MAX_DESCRIPTOR_DISTANCE;
    for (auto feature_idx:neighbour_features) {
      if (curr_frame.map_points[feature_idx] != nullptr &&
        curr_frame.map_points[feature_idx]->GetNumObservations() > 0)
      {
        continue;
      }

      if (curr_frame.stereo_key_points[feature_idx] > 0 &&
        std::abs(
          mp->projected_right -
          curr_frame.stereo_key_points[feature_idx]) > radius)
      {
        continue;
      }

      auto curr_descriptor = curr_frame.descriptors[feature_idx];
      auto dist = hamming_distance(mp_descriptor, curr_descriptor);
      if (dist < min_distance_1) {
        min_distance_idx = feature_idx;
        min_distance_level_2 = min_distance_level_1;
        min_distance_level_1 = curr_frame.key_points[feature_idx].octave;
        min_distance_2 = min_distance_1;
        min_distance_1 = dist;
      } else if (dist < min_distance_2) {
        min_distance_level_2 = curr_frame.key_points[feature_idx].octave;
        min_distance_2 = dist;
      }
    }

    if (min_distance_1 < MIN_DISTANCE_THRESHOLD) {
      if (min_distance_level_1 == min_distance_level_2 &&
        min_distance_1 > min_distance_2 * nn_dist_ratio_)
      {
        continue;
      }
      curr_frame.map_points[min_distance_idx] = mp;
      matches++;
    }
  }
  return matches;
}

int FeatureMatcher::EpipolarSearch(
  KeyFrame & kf1, KeyFrame & kf2, cv::Mat f_matrix,
  std::vector<std::pair<int, int>> & matching_pairs) const
{
  std::array<std::vector<int>, HISTOGRAM_BINS> rotation_histogram;
  auto valid_matches_1 = std::vector<bool>(kf1.key_points.size(), false);
  auto valid_matches_2 = std::vector<bool>(kf2.key_points.size(), false);
  auto matching_idxs = std::vector<int>(kf1.key_points.size(), -1);

  auto kf_feature_map_1 = kf1.feat_vector;
  auto kf_feature_map_2 = kf2.feat_vector;
  auto kf_it_1 = kf_feature_map_1.begin();
  auto kf_it_2 = kf_feature_map_2.begin();

  while (kf_it_1 != kf_feature_map_1.end() && kf_it_2 != kf_feature_map_2.end()) {
    auto [kf_id_1, kf_features_1] = *kf_it_1;
    auto [kf_id_2, kf_features_2] = *kf_it_2;
    if (kf_id_1 == kf_id_2) {
      for (auto idx1:kf_features_1) {
        auto mp1 = kf1.GetMapPoint(idx1);
        if (mp1 != nullptr) {
          continue;
        }

        auto kp1 = kf1.key_points[idx1];
        auto descriptor_1 = kf1.descriptors[idx1];
        int min_distance = MIN_DISTANCE_THRESHOLD;
        int min_distance_idx_2 = -1;
        for (auto idx2:kf_features_2) {
          auto mp2 = kf2.GetMapPoint(idx2);
          if (mp2 != nullptr || valid_matches_2[idx2]) {
            continue;
          }

          auto descriptor_2 = kf2.descriptors[idx2];
          auto dist = hamming_distance(descriptor_1, descriptor_2);
          if (dist > min_distance) {
            continue;
          }

          auto kp2 = kf2.key_points[idx2];
          if (validateEpipolarConstraints(kp1, kp2, f_matrix, kf2)) {
            min_distance = dist;
            min_distance_idx_2 = idx2;
          }
        }

        if (min_distance_idx_2 != -1) {
          matching_idxs[idx1] = min_distance_idx_2;
          valid_matches_1[idx1] = true;
          valid_matches_2[min_distance_idx_2] = true;

          auto kp2 = kf2.key_points[min_distance_idx_2];
          float rotation = kp1.angle - kp2.angle;
          if (rotation < 0.0) {
            rotation += 360.0;
          }
          int bin = int((rotation - 1) * HISTOGRAM_BINS / 360.0);
          rotation_histogram[bin].push_back(idx1);
        }
      }

      kf_it_1++;
      kf_it_2++;
    } else if (kf_id_1 < kf_id_2) {
      kf_it_1 = kf_feature_map_1.lower_bound(kf_id_2);
    } else {
      kf_it_2 = kf_feature_map_2.lower_bound(kf_id_1);
    }
  }

  int matches = applyHistogramFilter(rotation_histogram, valid_matches_1);
  for (int i = 0; i < int(matching_idxs.size()); i++) {
    if (valid_matches_1[i]) {
      matching_pairs.push_back(std::make_pair(i, matching_idxs[i]));
    }
  }
  return matches;
}

int FeatureMatcher::Fuse(
  std::shared_ptr<KeyFrame> kf,
  std::vector<std::shared_ptr<MapPoint>> & map_points,
  int search_radius_threshold) const
{
  auto fx = kf->camera_params.fx;
  auto fy = kf->camera_params.fy;
  auto cx = kf->camera_params.cx;
  auto cy = kf->camera_params.cy;
  auto kf_rotation_cw = kf->GetRotation();
  auto kf_translation_cw = kf->GetTranslation();
  auto kf_camera_cw = kf->GetCameraCenter();

  int num_fused = 0;
  for (auto mp:map_points) {
    if (mp == nullptr || mp->Culled() || mp->InKeyFrame(kf)) {
      continue;
    }

    auto mp_w = mp->GetWorldPos();
    // Project point onto key frame camera's frame.
    cv::Mat mp_c = kf_rotation_cw * mp_w + kf_translation_cw;
    auto depth = mp_c.at<float>(2);
    if (depth < 0.0) {
      continue;
    }

    // Project point onto key frame pixels frame.
    auto u = fx * mp_c.at<float>(0) / depth + cx;
    auto v = fy * mp_c.at<float>(1) / depth + cy;

    // Validate that point's distance from camera matches image scale level.
    auto [min_dist, max_dist] = mp->GetDistanceInvariance();
    auto mp_camera_w = mp_w - kf_camera_cw;
    auto camera_dist = cv::norm(mp_camera_w);
    if (camera_dist < min_dist || camera_dist > max_dist) {
      continue;
    }

    // Validate that viewing angle is less than 60 degrees.
    auto normal = mp->GetNormal();
    float cos = mp_camera_w.dot(normal) / camera_dist;
    if (cos < Frame::FRUSTUM_COSINE_THRESHOLD) {
      return false;
    }

    auto predicted_level = MapPoint::PredictScale(
      camera_dist, max_dist, kf->scale_factor, kf->image_scale_factors.size());
    auto radius = (search_radius_threshold *
      kf->image_scale_factors[predicted_level]);
    auto neighbour_features = kf->FindNeighbourFeatures(u, v, radius, 0);

    auto descriptor_mp = mp->GetDescriptor();
    int min_desc_dist = MAX_DESCRIPTOR_DISTANCE;
    int min_desc_dist_idx = -1;
    for (auto feature_idx:neighbour_features) {
      auto kp = kf->key_points[feature_idx];
      if (kp.octave < predicted_level - 1 || kp.octave > predicted_level) {
        continue;
      }

      auto descriptor_kf = kf->descriptors[feature_idx];
      auto hamming_dist = hamming_distance(descriptor_mp, descriptor_kf);
      if (hamming_dist < min_desc_dist) {
        hamming_dist = min_desc_dist;
        min_desc_dist_idx = feature_idx;
      }
    }

    if (min_desc_dist <= MIN_DISTANCE_THRESHOLD) {
      auto kf_mp = kf->GetMapPoint(min_desc_dist_idx);
      if (kf_mp != nullptr) {
        if (!kf_mp->Culled()) {
          if (kf_mp->GetNumObservations() > mp->GetNumObservations()) {
            mp->Replace(kf_mp);
          } else {
            kf_mp->Replace(mp);
          }
        }
      } else {
        mp->AddObservation(kf, min_desc_dist_idx);
        mp->UpdateObservations();
        kf->AddMapPoint(mp, min_desc_dist_idx);
      }
      num_fused++;
    }
  }

  return num_fused;
}

int FeatureMatcher::applyHistogramFilter(
  const std::array<std::vector<int>, HISTOGRAM_BINS> & rotation_histogram,
  std::vector<bool> & valid_matches) const
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
      valid_matches[point_idx] = false;
    }
  }
  return matches;
}

bool FeatureMatcher::validateEpipolarConstraints(
  const cv::KeyPoint & kp1, const cv::KeyPoint & kp2, const cv::Mat f_matrix,
  const KeyFrame & kf) const
{
  // Compute distance of key point from epipolar line.
  // For a line of the form ax+by+c=0 this is a(x1)+b(y1)+c/sqrt(a^2+b^2).
  auto a = (kp1.pt.x * f_matrix.at<float>(0, 0) +
    kp1.pt.y * f_matrix.at<float>(1, 0) + f_matrix.at<float>(2, 0));
  auto b = (kp1.pt.x * f_matrix.at<float>(0, 1) +
    kp1.pt.y * f_matrix.at<float>(1, 1) + f_matrix.at<float>(2, 1));
  auto c = (kp1.pt.x * f_matrix.at<float>(0, 2) +
    kp1.pt.y * f_matrix.at<float>(1, 2) + f_matrix.at<float>(2, 2));
  if (a == 0 && b == 0) {
    return false;
  }
  auto numerator = a * kp2.pt.x + b * kp2.pt.y + c;
  auto denominator = a * a + b * b;
  auto dist = numerator * numerator / denominator;

  auto scale_factor = (kf.image_scale_factors[kp2.octave] *
    kf.image_scale_factors[kp2.octave]);
  return dist < FUNDAMENTAL_MATRIX_OUTLIER_THRESHOLD * scale_factor;
}

}  // vslam
