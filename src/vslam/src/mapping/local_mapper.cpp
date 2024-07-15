#include "vslam/mapping/local_mapper.hpp"
#include "vslam/mapping/map.hpp"
#include "vslam/mapping/map_point.hpp"
#include "vslam/optimization/optimizer.hpp"
#include "vslam/tracking/feature_matcher.hpp"
#include "vslam/tracking/key_frame.hpp"

namespace vslam
{

LocalMapper::LocalMapper(std::shared_ptr<Map> map)
: map_(map)
{
}

void LocalMapper::Run()
{
  for (;; ) {
    {
      std::unique_lock lock(queue_mutex_);
      queue_cv_.wait(lock, [this] {return !kf_queue_.empty();});
      curr_kf_ = kf_queue_.front();
    }

    processCurrKeyFrame();
    cullMapPoints();
    triangulateNewMapPoints();
    if (KeyFramesQueued() == 0) {
      fuseDuplicateMapPoints();
    }
    if (KeyFramesQueued() == 0) {
      if (map_->GetKeyFrameCount() > 2) {
        Optimizer optimizer;
        optimizer.LocalBundleAdjustment(curr_kf_);
      }
      cullKeyFrames();
    }

    {
      std::lock_guard<std::mutex> lock(queue_mutex_);
      kf_queue_.pop();
    }
  }
}

void LocalMapper::EnqueueKeyFrame(std::shared_ptr<KeyFrame> kf)
{
  std::lock_guard<std::mutex> lock(queue_mutex_);
  kf_queue_.push(kf);
  queue_cv_.notify_one();
}

int LocalMapper::KeyFramesQueued()
{
  std::lock_guard<std::mutex> lock(queue_mutex_);
  return kf_queue_.size();
}

void LocalMapper::processCurrKeyFrame()
{
  auto map_points = curr_kf_->GetMapPoints();
  for (int i = 0; i < int(map_points.size()); i++) {
    auto mp = map_points[i];
    if (mp != nullptr && !mp->Culled()) {
      if (!mp->InKeyFrame(curr_kf_)) {
        mp->AddObservation(curr_kf_, i);
        mp->UpdateObservations();
      } else {
        local_map_points_.push(mp);
      }
    }
  }

  curr_kf_->ComputeBoW();
  curr_kf_->UpdateConnections();
  map_->AddKeyFrame(curr_kf_);
}

void LocalMapper::triangulateNewMapPoints()
{
  FeatureMatcher matcher(0.6);
  auto local_key_frames = curr_kf_->GetCovisibleKeyFrames(10);
  cv::Mat curr_camera_pos = curr_kf_->GetCameraCenter();

  auto kf_rotation_c1w = curr_kf_->GetRotation();
  auto kf_translation_c1w = curr_kf_->GetTranslation();
  auto kf_center_c1 = curr_kf_->GetCameraCenter();

  for (int i = 0; i < int(local_key_frames.size()); i++) {
    if (i > 0 && KeyFramesQueued() > 0) {
      return;
    }

    auto neighbour_kf = local_key_frames[i];
    cv::Mat neighbour_camera_pos = neighbour_kf->GetCameraCenter();
    auto baseline = cv::norm(curr_camera_pos - neighbour_camera_pos);
    if (baseline < curr_kf_->camera_params.depth_baseline) {
      continue;
    }

    auto kf_rotation_c2w = neighbour_kf->GetRotation();
    auto kf_translation_c2w = neighbour_kf->GetTranslation();
    auto kf_center_c2 = neighbour_kf->GetCameraCenter();

    auto f_matrix = computeFundamentalMatrix(
      curr_kf_.get(), neighbour_kf.get());
    std::vector<std::pair<int, int>> matching_pairs;
    matcher.EpipolarSearch(*curr_kf_, *neighbour_kf, f_matrix, matching_pairs);

    for (auto [idx1, idx2]:matching_pairs) {
      auto kp1 = curr_kf_->key_points[idx1];
      auto kp2 = neighbour_kf->key_points[idx2];

      auto world_pos = triangulateNewPoint(
        curr_kf_->camera_params, kp1, kp2, kf_rotation_c1w, kf_rotation_c2w,
        kf_translation_c1w, kf_translation_c2w);
      if (!world_pos) {
        continue;
      }

      if (!validateTriangulationVisible(
          *world_pos, kf_rotation_c1w, kf_translation_c1w) ||
        !validateTriangulationVisible(
          *world_pos, kf_rotation_c2w, kf_translation_c2w))
      {
        continue;
      }

      if (!validateReprojectionError(
          curr_kf_->camera_params, *world_pos, kp1,
          kf_rotation_c1w, kf_translation_c1w) ||
        !validateReprojectionError(
          curr_kf_->camera_params, *world_pos, kp2,
          kf_rotation_c2w, kf_translation_c2w))
      {
        continue;
      }

      if (!validateScaleConsistency(
          *world_pos, kp1, kp2, kf_center_c1, kf_center_c2))
      {
        continue;
      }

      auto mp = std::make_shared<MapPoint>(*world_pos, curr_kf_, map_);
      mp->AddObservation(curr_kf_, idx1);
      mp->AddObservation(neighbour_kf, idx2);
      curr_kf_->AddMapPoint(mp, idx1);
      neighbour_kf->AddMapPoint(mp, idx2);
      mp->UpdateObservations();

      map_->AddMapPoint(mp);
      local_map_points_.push(mp);
    }
  }
}

void LocalMapper::fuseDuplicateMapPoints()
{
  auto local_key_frames = curr_kf_->GetCovisibleKeyFrames(10);
  std::vector<std::shared_ptr<KeyFrame>> target_key_frames;
  for (auto kf:local_key_frames) {
    if (!kf->Culled() &&
      std::find(
        target_key_frames.begin(), target_key_frames.end(), kf
      ) == target_key_frames.end())
    {
      target_key_frames.push_back(kf);
    }

    auto neighbour_key_frames = kf->GetCovisibleKeyFrames(5);
    for (auto neighbour_kf:neighbour_key_frames) {
      if (!neighbour_kf->Culled() &&
        std::find(
          target_key_frames.begin(), target_key_frames.end(), kf
        ) == target_key_frames.end())
      {
        target_key_frames.push_back(neighbour_kf);
      }
    }
  }

  FeatureMatcher matcher(0.6);
  auto kf_map_points = curr_kf_->GetMapPoints();
  for (auto kf:target_key_frames) {
    matcher.Fuse(kf, kf_map_points);
  }

  std::vector<std::shared_ptr<MapPoint>> fuse_candidates;
  for (auto kf:target_key_frames) {
    auto target_map_points = kf->GetMapPoints();
    for (auto mp:target_map_points) {
      if (mp != nullptr && !mp->Culled() &&
        std::find(
          fuse_candidates.begin(), fuse_candidates.end(), mp
        ) == fuse_candidates.end())
      {
        fuse_candidates.push_back(mp);
      }
    }
  }
  matcher.Fuse(curr_kf_, fuse_candidates);

  auto new_map_points = curr_kf_->GetMapPoints();
  for (auto mp:new_map_points) {
    if (mp != nullptr && !mp->Culled()) {
      mp->UpdateObservations();
    }
  }
  curr_kf_->UpdateConnections();
}

void LocalMapper::cullKeyFrames()
{
  auto local_key_frames = curr_kf_->GetCovisibleKeyFrames();
  for (auto kf:local_key_frames) {
    // Avoid removing origin key frame computed during pose intialization.
    if (kf->curr_id == 0) {
      continue;
    }

    auto map_points = kf->GetMapPoints();
    int valid_mp_count = 0;
    int redundant_mp_count = 0;
    for (int i = 0; i < int(map_points.size()); i++) {
      auto mp = map_points[i];
      // Ignore culled points and far points.
      if (mp == nullptr || mp->Culled() || kf->depth_points[i] < 0 ||
        kf->depth_points[i] > kf->camera_params.depth_threshold)
      {
        continue;
      }

      valid_mp_count++;
      if (mp->GetNumObservations() <= 3) {
        continue;
      }
      auto kf_level = kf->key_points[i].octave;
      auto observations = mp->GetObservations();
      int valid_obs_count = 0;
      for (auto [kf_obs, idx]:observations) {
        if (kf_obs->curr_id == kf->curr_id) {
          continue;
        }

        auto kf_obs_level = kf_obs->key_points[idx].octave;
        if (kf_obs_level > kf_level + 1) {
          continue;
        }

        valid_obs_count++;
        if (valid_obs_count >= REDUNDANT_KEY_FRAME_OBSERVATIONS) {
          break;
        }
      }
      // A point is redundant if there are >= 3 key frames containing the point.
      if (valid_obs_count >= REDUNDANT_KEY_FRAME_OBSERVATIONS) {
        redundant_mp_count++;
      }
    }

    // Remove the key frame if 90% of the points are redundant.
    if (redundant_mp_count > REDUNDANT_POINT_RATIO * valid_mp_count) {
      kf->Cull();
    }
  }
}

void LocalMapper::cullMapPoints()
{
  auto curr_id = curr_kf_->curr_id;
  auto size = local_map_points_.size();
  for (int i = 0; i < int(size); i++) {
    auto mp = local_map_points_.front();
    local_map_points_.pop();

    if (mp == nullptr || mp->Culled()) {
      continue;
    }
    if (mp->FoundRatio() < 0.25 ||
      (curr_id - mp->initial_kf_id >= 2 && mp->GetNumObservations() <= 3))
    {
      mp->Cull();
      continue;
    }
    if (curr_id - mp->initial_kf_id >= 3) {
      continue;
    }

    local_map_points_.push(mp);
  }
}

cv::Mat LocalMapper::computeFundamentalMatrix(
  const KeyFrame * kf1, const KeyFrame * kf2) const
{
  auto rotation_c1w = kf1->GetRotation();
  auto translation_c1w = kf1->GetTranslation();
  auto rotation_c2w = kf2->GetRotation();
  auto translation_c2w = kf2->GetTranslation();

  auto rotation_12 = rotation_c1w * rotation_c2w.t();
  cv::Mat translation_12 = -rotation_12 * translation_c2w + translation_c1w;
  // Skew-symmetric matrix used to represent cross product as matrix
  // multiplication.
  auto translation_cross = (cv::Mat_<float>(3, 3) <<
    0, -translation_12.at<float>(2), translation_12.at<float>(1),
    translation_12.at<float>(2), 0, -translation_12.at<float>(0),
    -translation_12.at<float>(1), translation_12.at<float>(0), 0);

  auto k = kf1->camera_params.k;
  return k.t().inv() * translation_cross * rotation_12 * k.inv();
}

std::optional<cv::Mat> LocalMapper::triangulateNewPoint(
  const CameraParams & params,
  const cv::KeyPoint & kp1, const cv::KeyPoint & kp2,
  const cv::Mat & rotation_c1w, const cv::Mat & rotation_c2w,
  const cv::Mat & translation_c1w, const cv::Mat & translation_c2w) const
{
  // Project points from pixel to camera frame (using normalized homogeneous
  // coordinates).
  cv::Mat proj_c1 = (cv::Mat_<float>(3, 1) <<
    (kp1.pt.x - params.cx) / params.fx, (kp1.pt.y - params.cy) / params.fy, 1);
  cv::Mat proj_c2 = (cv::Mat_<float>(3, 1) <<
    (kp2.pt.x - params.cx) / params.fx, (kp2.pt.y - params.cy) / params.fy, 1);

  // Rotate points to match world coordinate frame.
  auto proj1_w = rotation_c1w.t() * proj_c1;
  auto proj2_w = rotation_c2w.t() * proj_c2;

  auto proj_cos = (proj1_w.dot(proj2_w) /
    (cv::norm(proj1_w) * cv::norm(proj2_w)));
  if (proj_cos < 0 || proj_cos > TRIANGULATION_COSINE_THRESHOLD) {
    return {};
  }

  // Estimate position by applying linear triangulation.
  cv::Mat A(4, 4, CV_32F);
  A.row(0) = proj_c1.at<float>(0) * rotation_c1w.row(2) - translation_c1w.row(0);
  A.row(1) = proj_c1.at<float>(1) * rotation_c1w.row(2) - translation_c1w.row(1);
  A.row(2) = proj_c2.at<float>(0) * rotation_c2w.row(2) - translation_c2w.row(0);
  A.row(3) = proj_c2.at<float>(1) * rotation_c2w.row(2) - translation_c2w.row(1);
  cv::Mat w, u, vt;
  cv::SVD::compute(A, w, u, vt, cv::SVD::MODIFY_A | cv::SVD::FULL_UV);

  // Find eigenvector associated with smallest eigenvalue.
  cv::Mat svd_pos = vt.row(3).t();
  if (svd_pos.at<float>(3) == 0) {
    return {};
  }
  auto world_pos = svd_pos.rowRange(0, 3) / svd_pos.at<float>(3);
  return world_pos;
}

bool LocalMapper::validateTriangulationVisible(
  const cv::Mat & world_pos,
  const cv::Mat & rotation_cw, const cv::Mat & translation_cw) const
{
  float depth = rotation_cw.row(2).dot(world_pos.t()) + translation_cw.at<float>(2);
  return depth > 0;
}

bool LocalMapper::validateReprojectionError(
  const CameraParams & params,
  const cv::Mat & world_pos, const cv::KeyPoint & kp,
  const cv::Mat & rotation_cw, const cv::Mat & translation_cw) const
{
  // Project point onto camera frame.
  cv::Mat camera_pos = rotation_cw * world_pos + translation_cw;
  // Project point onto pixel frame.
  float u = (params.fx * camera_pos.at<float>(0) /
    camera_pos.at<float>(2) + params.cx);
  float v = (params.fy * camera_pos.at<float>(1) /
    camera_pos.at<float>(2) + params.cy);

  float x_err = u - kp.pt.x;
  float y_err = v - kp.pt.y;
  float scale = curr_kf_->image_scale_factors[kp.octave];
  return x_err * x_err + y_err * y_err <
         REPROJECTION_ERROR_THRESHOLD * scale * scale;
}

bool LocalMapper::validateScaleConsistency(
  const cv::Mat & world_pos,
  const cv::KeyPoint & kp1, const cv::KeyPoint & kp2,
  const cv::Mat & center_c1, const cv::Mat & center_c2) const
{
  auto dist1 = cv::norm(world_pos - center_c1);
  auto dist2 = cv::norm(world_pos - center_c2);
  if (dist1 == 0 || dist2 == 0) {
    return false;
  }

  auto dist_ratio = dist1 / dist2;
  auto scale_ratio = (curr_kf_->image_scale_factors[kp1.octave] /
    curr_kf_->image_scale_factors[kp2.octave]);
  auto threshold_ratio = 1.5 * curr_kf_->scale_factor;
  return dist_ratio < scale_ratio * threshold_ratio &&
         dist_ratio * threshold_ratio > scale_ratio;
}

}  // vslam
