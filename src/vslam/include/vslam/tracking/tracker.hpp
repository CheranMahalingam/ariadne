#ifndef VSLAM__TRACKER_HPP_
#define VSLAM__TRACKER_HPP_

#include "vslam/utils.hpp"
#include "vslam/tracking/feature_extractor.hpp"

#include "DBoW3/DBoW3.h"
#include <opencv2/core.hpp>
#include <rclcpp/rclcpp.hpp>

#include <functional>

namespace vslam
{

class Canvas;
class Frame;
class KeyFrame;
class LocalMapper;
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
  static constexpr int MIN_CLOSE_MAP_POINTS = 100;
  static constexpr int MIN_MATCHING_POINTS_BOW = 15;
  static constexpr int MIN_MATCHING_POINTS_PROJECTION = 20;
  static constexpr int MIN_LOCAL_MAP_INLIERS = 30;
  static constexpr int MIN_TRACKING_INLIERS = 10;
  static constexpr int PROJECTION_SEARCH_RADIUS = 15;
  static constexpr int MAX_LOCAL_KEY_FRAMES = 80;

  Tracker(
    Canvas * canvas,
    LocalMapper * local_mapper,
    std::shared_ptr<Map> map,
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

  void updateReplacedPoints();

  bool trackUsingMotionModel();
  bool trackUsingReferenceFrame();
  void updatePrevFrame();

  bool trackLocalMap();
  void updateLocalMap();
  void findLocalMatches();

  bool keyFrameInclusionHeuristic();
  void insertKeyFrame();

  void createNewMapPoint(int point_idx);
  void createClosePoints(
    Frame * frame, std::function<void(int)> point_creation_func);

  CameraParams camera_params_;

  DBoW3::Vocabulary vocabulary_;

  std::shared_ptr<Frame> curr_frame_;
  std::shared_ptr<Frame> prev_frame_;
  std::shared_ptr<KeyFrame> kf_;
  int prev_kf_id_;

  int local_matching_inliers_;

  std::vector<std::shared_ptr<KeyFrame>> local_key_frames_;
  std::vector<std::shared_ptr<MapPoint>> local_map_points_;
  std::vector<std::shared_ptr<MapPoint>> temp_map_points_;

  std::shared_ptr<Map> map_;

  cv::Mat velocity_;
  SLAMState state_;

  std::vector<float> image_scale_factors_;

  std::unique_ptr<FeatureExtractor> extractor_;

  Canvas * canvas_;
  LocalMapper * local_mapper_;
};

}  // vslam

#endif  // VSLAM__TRACKER_HPP_
