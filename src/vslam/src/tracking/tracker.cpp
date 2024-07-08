#include "vslam/mapping/local_mapper.hpp"
#include "vslam/mapping/map.hpp"
#include "vslam/mapping/map_point.hpp"
#include "vslam/tracking/frame.hpp"
#include "vslam/tracking/feature_matcher.hpp"
#include "vslam/tracking/key_frame.hpp"
#include "vslam/tracking/tracker.hpp"

#include <opencv2/imgproc.hpp>

#include <map>
#include <ranges>

namespace vslam
{

Tracker::Tracker(
  LocalMapper * local_mapper,
  std::shared_ptr<Map> map,
  const DBoW3::Vocabulary & vocabulary,
  const FeatureExtractor::ORBParams & orb_params,
  const CameraParams & camera_params)
: camera_params_(camera_params),
  vocabulary_(vocabulary),
  local_matching_inliers_(0),
  map_(map),
  state_(SLAMState::INITIALIZING),
  local_mapper_(local_mapper)
{
  extractor_ = std::make_unique<FeatureExtractor>(orb_params);
}

void Tracker::Track(
  const cv::Mat & rgb, const cv::Mat & depth, rclcpp::Time timestamp)
{
  auto grey = rgb;
  cv::cvtColor(grey, grey, cv::COLOR_RGB2GRAY);

  if (depth.type() != CV_32F) {
    depth.convertTo(depth, CV_32F, camera_params_.depth_map_factor);
  }

  std::vector<cv::KeyPoint> key_points;
  cv::Mat descriptors;
  extractor_->ComputeFeatures(grey, key_points, descriptors);

  curr_frame_ = std::make_shared<Frame>(
    depth, timestamp, key_points, descriptors,
    extractor_.get(), vocabulary_, camera_params_);

  if (state_ == SLAMState::INITIALIZING) {
    if (key_points.size() >= MIN_POINTS_POSE_INIT) {
      initializePose();
    }
    if (state_ != SLAMState::HEALTHY) {
      return;
    }
  } else {
    bool ok = false;
    if (velocity_.empty()) {
      ok = trackUsingReferenceFrame();
    } else {
      ok = trackUsingMotionModel();
    }

    ok = trackLocalMap();

    if (ok) {
      if (prev_frame_->pose.initialized) {
        velocity_ = curr_frame_->pose.transform_cw * prev_frame_->pose.transform_wc;
      } else {
        velocity_ = cv::Mat();
      }

      if (keyFrameInclusionHeuristic()) {
        insertKeyFrame();
      }
    }

    if (!ok) {
      state_ = SLAMState::LOST;
    } else {
      state_ = SLAMState::HEALTHY;
    }

    if (curr_frame_->kf_ref == nullptr) {
      curr_frame_->kf_ref = kf_;
    }
    prev_frame_ = std::make_shared<Frame>(*curr_frame_);
  }

  if (curr_frame_->pose.initialized) {
    cv::Mat kf_relative_transform = (curr_frame_->pose.transform_cw *
      curr_frame_->kf_ref->GetPoseInverse());
    pose_history.push_back(kf_relative_transform);
    frame_history.push_back(kf_);
  } else {
    pose_history.push_back(pose_history.back());
    frame_history.push_back(frame_history.back());
  }
  state_history.push_back(state_);
}

void Tracker::initializePose()
{
  curr_frame_->SetPose(cv::Mat::eye(4, 4, CV_32F));

  kf_ = std::make_shared<KeyFrame>(*curr_frame_, map_);
  map_->AddKeyFrame(kf_);

  for (int i = 0; i < int(curr_frame_->key_points.size()); i++) {
    createNewMapPoint(i);
  }

  local_mapper_->EnqueueKeyFrame(kf_);

  prev_frame_ = std::make_shared<Frame>(*curr_frame_);
  prev_kf_id_ = curr_frame_->curr_id;
  map_->origin = kf_;

  state_ = SLAMState::HEALTHY;
}

bool Tracker::trackUsingMotionModel()
{
  updatePrevFrame();

  curr_frame_->SetPose(velocity_ * prev_frame_->pose.transform_cw);
  std::fill(curr_frame_->map_points.begin(), curr_frame_->map_points.end(), nullptr);

  FeatureMatcher matcher(0.9);
  auto matches = matcher.ProjectionSearch(
    *curr_frame_, *prev_frame_, PROJECTION_SEARCH_RADIUS);
  // If the previous search failed, rerun with a larger radius.
  if (matches < MIN_MATCHING_POINTS_PROJECTION) {
    std::fill(curr_frame_->map_points.begin(), curr_frame_->map_points.end(), nullptr);
    matches = matcher.ProjectionSearch(
      *curr_frame_, *prev_frame_, 2 * PROJECTION_SEARCH_RADIUS);
  }
  if (matches < MIN_MATCHING_POINTS_PROJECTION) {
    return false;
  }

  // TODO: Optimize pose

  int inlier_matches = 0;
  return inlier_matches >= MIN_TRACKING_INLIERS;
}

bool Tracker::trackUsingReferenceFrame()
{
  curr_frame_->ComputeBoW();

  FeatureMatcher matcher(0.7);
  std::vector<std::shared_ptr<MapPoint>> matching_points(
    int(curr_frame_->key_points.size()), nullptr);
  auto matches = matcher.BoWSearch(*kf_, *curr_frame_, matching_points);
  if (matches < MIN_MATCHING_POINTS_BOW) {
    return false;
  }

  curr_frame_->map_points = std::move(matching_points);
  curr_frame_->SetPose(prev_frame_->pose.transform_cw);

  // TODO: Optimize pose

  int inlier_matches = 0;
  return inlier_matches >= MIN_TRACKING_INLIERS;
}

void Tracker::updatePrevFrame()
{
  auto prev_kf_ref = prev_frame_->kf_ref;
  cv::Mat prev_transform = pose_history.back();
  prev_frame_->SetPose(prev_transform * prev_kf_ref->GetPose());

  if (prev_kf_id_ == prev_frame_->curr_id) {
    return;
  }

  auto create_point = [this](int idx) -> void {
      auto world_pos = prev_frame_->UnprojectToWorldFrame(idx);
      auto new_map_point = std::make_shared<MapPoint>(
        world_pos, kf_, map_);
      prev_frame_->map_points[idx] = new_map_point;
    };
  createClosePoints(prev_frame_.get(), create_point);
}

bool Tracker::trackLocalMap()
{
  updateLocalMap();
  findLocalMatches();

  // TODO: Optimize pose

  local_matching_inliers_ = 0;
  for (auto mp:curr_frame_->map_points) {
    if (mp == nullptr) {
      continue;
    }
    if (mp->GetNumObservations() > 0) {
      local_matching_inliers_++;
    }
    mp->IncreaseFound();
  }
  return local_matching_inliers_ >= MIN_LOCAL_MAP_INLIERS;
}

void Tracker::updateLocalMap()
{
  int max_matching_points = 0;
  std::shared_ptr<KeyFrame> similar_kf;

  // Find all key frames tracking similar points to the current frame.
  std::map<std::shared_ptr<KeyFrame>, int> key_frame_obs;
  for (auto mp:curr_frame_->map_points) {
    if (mp == nullptr || mp->Culled()) {
      mp = nullptr;
      continue;
    }
    for (auto [kf, _]:mp->GetObservations()) {
      key_frame_obs[kf]++;
    }
  }

  if (key_frame_obs.empty()) {
    return;
  }
  local_key_frames_.clear();

  for (auto [kf, count]:key_frame_obs) {
    if (kf->Culled()) {
      continue;
    }
    // Track the maximum matching points across all key frames to set the
    // reference frame for the current frame.
    if (count > max_matching_points) {
      max_matching_points = count;
      similar_kf = kf;
    }
    local_key_frames_.push_back(kf);
  }

  // Include additional neighbouring key frames from the covisibility graph and
  // spanning tree.
  auto new_local_key_frames = local_key_frames_;
  for (auto kf:local_key_frames_) {
    if (new_local_key_frames.size() > MAX_LOCAL_KEY_FRAMES) {
      break;
    }

    auto neighbours = kf->GetCovisibleKeyFrames(10);
    for (auto neighbour:neighbours) {
      if (!neighbour->Culled() &&
        std::find(
          new_local_key_frames.begin(),
          new_local_key_frames.end(),
          neighbour
        ) == new_local_key_frames.end())
      {
        new_local_key_frames.push_back(neighbour);
        break;
      }
    }

    auto sp_childs = kf->GetChildren();
    for (auto child:sp_childs) {
      if (!child->Culled() &&
        std::find(
          new_local_key_frames.begin(), new_local_key_frames.end(), child
        ) == new_local_key_frames.end())
      {
        new_local_key_frames.push_back(child);
        break;
      }
    }

    auto sp_parent = kf->GetParent();
    if (!sp_parent->Culled() &&
      std::find(
        new_local_key_frames.begin(), new_local_key_frames.end(), sp_parent
      ) == new_local_key_frames.end())
    {
      new_local_key_frames.push_back(sp_parent);
    }
  }

  if (similar_kf != nullptr) {
    curr_frame_->kf_ref = similar_kf;
  }
  local_key_frames_ = new_local_key_frames;

  local_map_points_.clear();
  // Track all points associated with local key frames.
  for (auto kf:local_key_frames_) {
    auto map_points = kf->GetMapPoints();
    for (auto mp:map_points) {
      if (mp == nullptr ||
        std::find(
          local_map_points_.begin(), local_map_points_.end(), mp
        ) == local_map_points_.end())
      {
        continue;
      }
      local_map_points_.push_back(mp);
    }
  }
}

void Tracker::findLocalMatches()
{
  for (auto mp:curr_frame_->map_points) {
    if (mp->Culled()) {
      mp = nullptr;
    } else {
      mp->tracked = false;
      mp->IncreaseVisible();
    }
  }

  int matches = 0;
  for (auto local_mp:local_map_points_) {
    if (local_mp->Culled() ||
      std::find(
        curr_frame_->map_points.begin(), curr_frame_->map_points.end(), local_mp
      ) != curr_frame_->map_points.end())
    {
      continue;
    }

    if (curr_frame_->ValidFrustumProjection(local_mp)) {
      local_mp->IncreaseVisible();
      matches++;
    }
  }

  if (matches > 0) {
    FeatureMatcher matcher(0.8);
    matcher.ProjectionSearch(*curr_frame_, local_map_points_);
  }
}

bool Tracker::keyFrameInclusionHeuristic()
{
  int key_frame_count = map_->GetKeyFrameCount();
  if (key_frame_count >= camera_params_.fps) {
    return false;
  }

  int num_tracked_points = kf_->GetNumTrackedPoints(3);
  int tracked_close_points = 0;
  int non_tracked_close_points = 0;
  for (int i = 0; i < int(curr_frame_->map_points.size()); i++) {
    if (curr_frame_->depth_points[i] > 0 &&
      curr_frame_->depth_points[i] < camera_params_.depth_threshold)
    {
      if (curr_frame_->map_points[i] != nullptr) {
        tracked_close_points++;
      } else {
        non_tracked_close_points++;
      }
    }
  }

  /**
   * Condition 1a: More than 20 frames have passed since the last key frame
   * insertion.
   * Condition 1b: Local mapping is idle.
   * Condition 1c: Too few close points are being tracked or too few points in
   * the local map are being observed.
   * Condition 2: Mandatory condition with more liberal constraints than 1c.
   */
  bool condition_1a = curr_frame_->curr_id >= prev_kf_id_ + camera_params_.fps;
  bool condition_1b = local_mapper_->KeyFramesQueued() == 0;
  bool close_point_deficient = tracked_close_points < 100 && non_tracked_close_points > 70;
  bool condition_1c = (local_matching_inliers_ < 0.25 * num_tracked_points ||
    close_point_deficient);
  bool condition_2 = ((local_matching_inliers_ < 0.75 * num_tracked_points ||
    close_point_deficient) && local_matching_inliers_ > 15);
  if ((condition_1a || condition_1b || condition_1c) && condition_2) {
    if (local_mapper_->KeyFramesQueued() < 3) {
      return true;
    } else {
      return false;
    }
  } else {
    return false;
  }
}

void Tracker::insertKeyFrame()
{
  kf_ = std::make_shared<KeyFrame>(*curr_frame_, map_);
  curr_frame_->kf_ref = kf_;
  createClosePoints(
    curr_frame_.get(), std::bind(
      &Tracker::createNewMapPoint, this, std::placeholders::_1));

  local_mapper_->EnqueueKeyFrame(kf_);
  prev_kf_id_ = curr_frame_->curr_id;
}

void Tracker::createNewMapPoint(int point_idx)
{
  auto world_pos = curr_frame_->UnprojectToWorldFrame(point_idx);
  auto new_map_point = std::make_shared<MapPoint>(world_pos, kf_, map_);
  new_map_point->AddObservation(kf_, point_idx);
  new_map_point->UpdateObservations();
  kf_->AddMapPoint(new_map_point, point_idx);
  map_->AddMapPoint(new_map_point);
  curr_frame_->map_points[point_idx] = new_map_point;
}

void Tracker::createClosePoints(
  Frame * frame, std::function<void(int)> point_creation_func)
{
  std::vector<std::pair<int, int>> depths;
  for (int i = 0; i < int(frame->depth_points.size()); i++) {
    depths.push_back(std::make_pair(frame->depth_points[i], i));
  }
  std::ranges::sort(depths);

  int close_points = 0;
  for (auto [dist, idx]:depths) {
    auto mp = frame->map_points[idx];
    if (mp == nullptr || mp->GetNumObservations() < 1) {
      point_creation_func(idx);
    }

    close_points++;
    if (close_points >= MIN_CLOSE_MAP_POINTS &&
      dist > camera_params_.depth_threshold)
    {
      break;
    }
  }
}

}  // vslam
