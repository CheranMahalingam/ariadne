#ifndef VSLAM__TRACKER_HPP_
#define VSLAM__TRACKER_HPP_

#include "vslam/camera_utils.hpp"
#include "vslam/tracking/feature_extractor.hpp"

#include "DBoW3/DBoW3.h"
#include <opencv2/core.hpp>
#include <rclcpp/rclcpp.hpp>

namespace vslam
{

class Tracker
{
public:
  enum class SLAMState
  {
    INITIALIZING,
    HEALTHY,
    LOST,
  };

  static constexpr int MIN_POINTS_POSE_INIT = 500;

  Tracker(
    const DBoW3::Vocabulary & vocabulary,
    const FeatureExtractor::ORBParams & orb_params,
    const CameraParams & camera_params,
    double depth_map_factor,
    float nn_dist_ratio);

  void Track(
    const cv::Mat & rgb, const cv::Mat & depth, rclcpp::Time timestamp);

private:
  void initializePose();

  std::unique_ptr<FeatureExtractor> extractor_;

  DBoW3::Vocabulary vocabulary_;
  CameraParams camera_params_;
  double depth_map_factor_;
  float nn_dist_ratio_;

  SLAMState state_;
};

}  // vslam

#endif  // VSLAM__TRACKER_HPP_
