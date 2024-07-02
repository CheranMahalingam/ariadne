#ifndef VSLAM__FRAME_HPP_
#define VSLAM__FRAME_HPP_

#include "vslam/camera_utils.hpp"

#include "DBoW3/DBoW3.h"
#include <opencv2/core.hpp>
#include <rclcpp/rclcpp.hpp>

#include <vector>

namespace vslam
{

class KeyFrame;
class MapPoint;

class Frame
{
public:
  static constexpr int BOW_LEVELS = 4;
  static constexpr int FRAME_GRID_SIZE = 10;

  static long int frame_id;

  Frame(const Frame & frame);

  Frame(
    const cv::Mat & grey, const cv::Mat & depth, rclcpp::Time timestamp,
    const std::vector<cv::KeyPoint> & key_points, const cv::Mat & desc,
    const DBoW3::Vocabulary & vocabulary,
    const std::vector<float> & scale_factors,
    const CameraParams & camera_params);

  void ComputeBoW();

  cv::Mat UnprojectToWorldFrame(int point_idx) const;

  std::vector<int> FindNeighbourFeatures(float x, float y, float radius) const;

  void SetPose(cv::Mat pose);

  long int curr_id;

  CameraParams camera_params;

  std::vector<cv::KeyPoint> key_points;
  std::vector<float> stereo_key_points;
  std::vector<float> depth_points;
  std::vector<cv::Mat> descriptors;

  cv::Mat camera_world_transform;
  cv::Mat camera_world_rotation;
  cv::Mat camera_world_translation;
  cv::Mat world_pos;

  DBoW3::BowVector bow_vector;
  DBoW3::FeatureVector feat_vector;
  DBoW3::Vocabulary vocabulary;

  std::vector<float> image_scale_factors;

  std::shared_ptr<KeyFrame> kf_ref;

  std::vector<std::shared_ptr<MapPoint>> map_points;

  std::vector<std::vector<std::vector<int>>> frame_grid;

private:
  void populateGrid();

  void computeStereo(const cv::Mat & depth);
};

}  // vslam

#endif  // VSLAM__FRAME_HPP_
