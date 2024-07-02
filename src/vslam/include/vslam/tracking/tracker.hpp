#ifndef VSLAM__TRACKER_HPP_
#define VSLAM__TRACKER_HPP_

#include "vslam/camera_utils.hpp"
#include "vslam/tracking/feature_extractor.hpp"

#include "DBoW3/DBoW3.h"
#include <opencv2/core.hpp>
#include <rclcpp/rclcpp.hpp>

namespace vslam
{

class Frame;
class KeyFrame;
class Map;

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
  static constexpr int MIN_MATCHING_POINTS_BOW = 15;
  static constexpr int MIN_MATCHING_POINTS_PROJECTION = 20;
  static constexpr int PROJECTION_SEARCH_RADIUS = 15;

  Tracker(
    const DBoW3::Vocabulary & vocabulary,
    const FeatureExtractor::ORBParams & orb_params,
    const CameraParams & camera_params);

  void Track(
    const cv::Mat & rgb, const cv::Mat & depth, rclcpp::Time timestamp);

  std::vector<cv::Mat> pose_history;
  std::vector<std::shared_ptr<KeyFrame>> frame_history;
  std::vector<SLAMState> state_history;

private:
  void initializePose();

  bool trackUsingMotionModel();
  bool trackUsingReferenceFrame();

  CameraParams camera_params_;

  DBoW3::Vocabulary vocabulary_;

  std::shared_ptr<Frame> curr_frame_;
  std::shared_ptr<Frame> prev_frame_;
  std::shared_ptr<KeyFrame> kf_;
  int prev_kf_id_;

  std::shared_ptr<Map> map_;

  cv::Mat velocity_;
  SLAMState state_;

  std::vector<float> image_scale_factors_;

  std::unique_ptr<FeatureExtractor> extractor_;
};

}  // vslam

#endif  // VSLAM__TRACKER_HPP_
