#include "vslam/mapping/map_point.hpp"
#include "vslam/tracking/tracker.hpp"
#include "vslam/tracking/frame.hpp"

#include <opencv2/imgproc.hpp>

namespace vslam
{

Tracker::Tracker(
  const DBoW3::Vocabulary & vocabulary,
  const FeatureExtractor::ORBParams & orb_params,
  const CameraParams & camera_params,
  double depth_map_factor)
: vocabulary_(vocabulary),
  camera_params_(camera_params),
  depth_map_factor_(depth_map_factor),
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
    depth.convertTo(depth, CV_32F, depth_map_factor_);
  }

  std::vector<cv::KeyPoint> key_points;
  cv::Mat descriptors;
  extractor_->ComputeFeatures(grey, key_points, descriptors);

  cf_ = std::make_shared<Frame>(
    grey, depth, timestamp, key_points, descriptors, vocabulary_,
    camera_params_);

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
    }
  }
}

void Tracker::initializePose()
{
  cf_->SetPose(cv::Mat::eye(4, 4, CV_32F));

  kf_ = std::make_shared<KeyFrame>(cf_);
  map_->AddKeyFrame(kf_);

  for (int i = 0; i < cf_->GetSize(); i++) {
    auto world_pos = cf_->UnprojectToWorldFrame(i);
    std::shared_ptr<MapPoint> new_map_point = std::make_shared<MapPoint>(world_pos);
    map_->AddMapPoint(new_map_point);
  }

  state_ = SLAMState::HEALTHY;
}

bool Tracker::trackUsingMotionModel()
{
  return false;
}

bool Tracker::trackUsingReferenceFrame()
{
  cf_->ComputeBoW();

  FeatureMatcher matcher(0.7);
  std::vector<MapPoint> matching_points;
  auto matches = matcher.BoWSearch(*kf_, *cf_, matching_points);
  return matches < MIN_MATCHING_POINTS;
}

}  // vslam
