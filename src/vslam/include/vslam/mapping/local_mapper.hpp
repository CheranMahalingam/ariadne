#ifndef VSLAM__LOCAL_MAPPER_HPP_
#define VSLAM__LOCAL_MAPPER_HPP_

#include "vslam/utils.hpp"

#include <opencv2/core.hpp>

#include <condition_variable>
#include <memory>
#include <mutex>
#include <optional>
#include <queue>

namespace vslam
{

class KeyFrame;
class Map;
class MapPoint;

class LocalMapper
{
public:
  static constexpr float REDUNDANT_POINT_RATIO = 0.9;
  static constexpr int REDUNDANT_KEY_FRAME_OBSERVATIONS = 3;
  // Maximum cosine between lines to reliably triangulate a new point (1 degree).
  static constexpr float TRIANGULATION_COSINE_THRESHOLD = 0.9998;
  static constexpr float REPROJECTION_ERROR_THRESHOLD = 5.991;

  LocalMapper(std::shared_ptr<Map> map);

  void Run();

  void EnqueueKeyFrame(std::shared_ptr<KeyFrame> kf);
  int KeyFramesQueued();

private:
  void processCurrKeyFrame();

  void createNewMapPoints();

  void fuseDuplicateMapPoints();

  void cullKeyFrames();
  void cullMapPoints();

  /**
   * Constructs the fundamental matrix to compute the epipolar line in one frame
   * given a known point in the other frame.
   */
  cv::Mat computeFundamentalMatrix(
    const KeyFrame * kf1, const KeyFrame * kf2) const;

  /**
   * Computes the position of a new point in the world frame using two points
   * captured in different frames that satisfy the epipolar constraint. Returns
   * an empty value if triangulation is not feasible due to limited translation.
   */
  std::optional<cv::Mat> triangulateNewPoint(
    const CameraParams & params,
    const cv::KeyPoint & kp1, const cv::KeyPoint & kp2,
    const cv::Mat & rotation_c1w, const cv::Mat & rotation_c2w,
    const cv::Mat & translation_c1w, const cv::Mat & translation_c2w) const;

  /**
   * For a new map point created using triangulation, validate that it appears
   * in front of the camera.
   */
  bool validateTriangulationVisible(
    const cv::Mat & world_pos,
    const cv::Mat & rotation_cw, const cv::Mat & translation_cw) const;
  /**
   * Validate that when a triangulated point is projected onto the pixel frame
   * the distance to the original map point is below a specified threshold.
   */
  bool validateReprojectionError(
    const CameraParams & params,
    const cv::Mat & world_pos, const cv::KeyPoint & kp,
    const cv::Mat & rotation_cw, const cv::Mat & translation_cw) const;
  /**
   * Validate that the triangulated point's distance from each original frame
   * is consistent with the scale of each key point.
   */
  bool validateScaleConsistency(
    const cv::Mat & world_pos,
    const cv::KeyPoint & kp1, const cv::KeyPoint & kp2,
    const cv::Mat & center_c1, const cv::Mat & center_c2) const;

  std::mutex queue_mutex_;
  std::condition_variable queue_cv_;
  std::queue<std::shared_ptr<KeyFrame>> kf_queue_;

  std::shared_ptr<KeyFrame> curr_kf_;

  std::queue<std::shared_ptr<MapPoint>> local_map_points_;
  std::shared_ptr<Map> map_;
};

}  // vslam

#endif  // VSLAM__LOCAL_MAPPER_HPP_
