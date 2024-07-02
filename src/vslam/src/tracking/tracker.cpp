#include "vslam/mapping/map.hpp"
#include "vslam/mapping/map_point.hpp"
#include "vslam/tracking/frame.hpp"
#include "vslam/tracking/feature_matcher.hpp"
#include "vslam/tracking/key_frame.hpp"
#include "vslam/tracking/tracker.hpp"

#include <opencv2/imgproc.hpp>

namespace vslam
{

Tracker::Tracker(
  const DBoW3::Vocabulary & vocabulary,
  const FeatureExtractor::ORBParams & orb_params,
  const CameraParams & camera_params)
: camera_params_(camera_params),
  vocabulary_(vocabulary),
  state_(SLAMState::INITIALIZING)
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
    grey, depth, timestamp, key_points, descriptors, vocabulary_,
    extractor_->GetScaleFactors(), camera_params_);

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
    if (!ok) {
      state_ = SLAMState::LOST;
    } else {
      state_ = SLAMState::HEALTHY;
    }

    if (ok) {
      if (!prev_frame_->camera_world_transform.empty()) {
        cv::Mat prev_pose = cv::Mat::eye(4, 4, CV_32F);
        cv::Mat world_camera_rotation = prev_frame_->camera_world_rotation.t();
        world_camera_rotation.copyTo(prev_pose.rowRange(0, 3).colRange(0, 3));
        prev_frame_->world_pos.copyTo(prev_pose.rowRange(0, 3).col(3));
        velocity_ = curr_frame_->camera_world_transform * prev_pose;
      } else {
        velocity_ = cv::Mat();
      }
    }

    curr_frame_->kf_ref = kf_;
    prev_frame_ = std::make_shared<Frame>(*curr_frame_);
  }

  if (!curr_frame_->camera_world_transform.empty()) {
    cv::Mat camera_ref_transform = curr_frame_->camera_world_transform * \
      curr_frame_->kf_ref->GetPoseInverse();
    pose_history.push_back(camera_ref_transform);
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
    auto world_pos = curr_frame_->UnprojectToWorldFrame(i);
    std::shared_ptr<MapPoint> new_map_point = std::make_shared<MapPoint>(
      world_pos, kf_, map_);
    kf_->AddMapPoint(new_map_point, i);
    curr_frame_->map_points[i] = new_map_point;
    map_->AddMapPoint(new_map_point);
  }

  prev_frame_ = std::make_shared<Frame>(*curr_frame_);
  prev_kf_id_ = curr_frame_->curr_id;

  state_ = SLAMState::HEALTHY;
}

bool Tracker::trackUsingMotionModel()
{
  curr_frame_->SetPose(velocity_ * prev_frame_->camera_world_transform);
  std::fill(curr_frame_->map_points.begin(), curr_frame_->map_points.end(), nullptr);

  auto prev_kf_ref = prev_frame_->kf_ref;
  cv::Mat prev_transform = pose_history.back();
  prev_frame_->SetPose(prev_transform * prev_kf_ref->GetPose());
  // TODO: Temporarily add additional close map points to previous frame for
  // better feature matching and remove afterwards.

  FeatureMatcher matcher(0.9);
  auto matches = matcher.ProjectionSearch(
    *curr_frame_, *prev_frame_, PROJECTION_SEARCH_RADIUS);
  if (matches < MIN_MATCHING_POINTS_PROJECTION) {
    std::fill(curr_frame_->map_points.begin(), curr_frame_->map_points.end(), nullptr);
    matches = matcher.ProjectionSearch(
      *curr_frame_, *prev_frame_, 2 * PROJECTION_SEARCH_RADIUS);
  }
  if (matches < MIN_MATCHING_POINTS_PROJECTION) {
    return false;
  }

  return true;
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
  curr_frame_->SetPose(prev_frame_->camera_world_transform);

  return true;
}

}  // vslam
