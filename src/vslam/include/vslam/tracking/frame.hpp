#ifndef VSLAM__FRAME_HPP_
#define VSLAM__FRAME_HPP_

#include "vslam/utils.hpp"

#include "DBoW3/DBoW3.h"
#include <opencv2/core.hpp>
#include <rclcpp/rclcpp.hpp>

#include <vector>

namespace vslam
{

class FeatureExtractor;
class KeyFrame;
class MapPoint;

class FrameBase
{
public:
  static constexpr int BOW_LEVELS = 4;
  static constexpr int FRAME_GRID_SIZE = 10;

  FrameBase(
    const cv::Mat & depth,
    const std::vector<cv::KeyPoint> & key_points,
    const cv::Mat & desc,
    const FeatureExtractor * extractor,
    const DBoW3::Vocabulary & vocabulary,
    const CameraParams & camera_params);
  FrameBase(const FrameBase & frame);
  virtual ~FrameBase();

  void ComputeBoW();

  std::vector<int> FindNeighbourFeatures(
    float x, float y, float radius, int min_level) const;
  std::vector<int> FindNeighbourFeatures(
    float x, float y, float radius, int min_level, int max_level) const;

  /**
   * Projects a key point onto the world coordinate frame. Points are stored in
   * the pixel coordinate frame (u,v) with an associated depth, z. This can be
   * projected to the camera frame using x = (u - cx) * z / fx and
   * y = (v - cy) * z / fy. Finally, this is projected onto the world frame
   * using the world to camera transformation matrix.
   */
  virtual cv::Mat UnprojectToWorldFrame(int point_idx) const = 0;

  CameraParams camera_params;

  std::vector<cv::KeyPoint> key_points;
  std::vector<float> stereo_key_points;
  std::vector<float> depth_points;
  std::vector<cv::Mat> descriptors;

  DBoW3::BowVector bow_vector;
  DBoW3::FeatureVector feat_vector;
  DBoW3::Vocabulary vocabulary;

  float scale_factor;
  std::vector<float> image_scale_factors;

  std::vector<std::vector<std::vector<int>>> grid;

protected:
  cv::Mat unprojectKeyPoint(int point_idx, Pose pose) const;

private:
  void populateGrid();

  void computeStereo(const cv::Mat & depth);
};

class Frame : public FrameBase
{
public:
  static constexpr float FRUSTUM_COSINE_THRESHOLD = 0.5;

  static long int frame_id;

  Frame(const Frame & frame);

  Frame(
    const cv::Mat & depth,
    rclcpp::Time timestamp,
    const std::vector<cv::KeyPoint> & key_points,
    const cv::Mat & descriptors,
    const FeatureExtractor * extractor,
    const DBoW3::Vocabulary & vocabulary,
    const CameraParams & camera_params);

  cv::Mat UnprojectToWorldFrame(int point_idx) const override;

  /**
   * Validates whether a map point lies within the camera's FOV (frustum).
   * Performs checks that distance from camera is within a bound and the angle
   * between the point and mean normal is small (limit frustum scope).
   */
  bool ValidFrustumProjection(std::shared_ptr<MapPoint> mp) const;

  void SetPose(cv::Mat pose_cw);

  long int curr_id;

  Pose pose;

  std::shared_ptr<KeyFrame> kf_ref;

  std::vector<std::shared_ptr<MapPoint>> map_points;
};

}  // vslam

#endif  // VSLAM__FRAME_HPP_
