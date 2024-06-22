#include "vslam/tracking/tracker.hpp"
#include "vslam/tracking/feature_matcher.hpp"
#include "vslam/tracking/frame.hpp"

#include <opencv2/imgproc.hpp>

namespace vslam
{

Tracker::Tracker(
  const DBoW3::Vocabulary & vocabulary,
  const FeatureExtractor::ORBParams & orb_params,
  const CameraParams & camera_params,
  double depth_map_factor,
  float nn_dist_ratio)
: vocabulary_(vocabulary),
  camera_params_(camera_params),
  depth_map_factor_(depth_map_factor),
  nn_dist_ratio_(nn_dist_ratio),
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

  auto frame = Frame(grey, depth, timestamp, key_points, descriptors, vocabulary_, camera_params_);

  if (state_ == SLAMState::INITIALIZING) {
    if (key_points.size() < MIN_POINTS_POSE_INIT) {
      return;
    }

    initializePose();

    state_ = SLAMState::HEALTHY;
  } else {
    frame.ComputeBoW();

    FeatureMatcher matcher(nn_dist_ratio_);
    // TODO: Search for matches with the reference key frame. If there are
    // sufficient matches, add frame to map.
  }
}

// TODO: Set pose to origin and initialize key frame in map.
void Tracker::initializePose()
{
}

}  // vslam
