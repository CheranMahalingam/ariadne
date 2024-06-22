#ifndef VSLAM__FRAME_HPP_
#define VSLAM__FRAME_HPP_

#include "vslam/camera_utils.hpp"
#include "vslam/mapping/map.hpp"

#include "DBoW3/DBoW3.h"
#include <opencv2/core.hpp>
#include <rclcpp/rclcpp.hpp>

#include <vector>

namespace vslam
{

class Frame
{
public:
  static constexpr int BOW_LEVELS = 4;

  static long int frame_id;

  Frame(
    const cv::Mat & grey, const cv::Mat & depth, rclcpp::Time timestamp,
    const std::vector<cv::KeyPoint> & key_points, const cv::Mat & descriptors,
    const DBoW3::Vocabulary & vocabulary,
    const CameraParams & camera_params);

  void ComputeBoW();

private:
  void populateGrid();

  void computeStereo(const cv::Mat & depth);

  long int curr_id_;

  std::vector<cv::KeyPoint> key_points_;
  std::vector<float> stereo_key_points_;
  std::vector<float> key_point_depth_;
  std::vector<cv::Mat> descriptors_;

  DBoW3::BowVector bow_vector_;
  DBoW3::FeatureVector feat_vector_;
  DBoW3::Vocabulary vocabulary_;

  CameraParams camera_params_;

  std::vector<MapPoint> points_;
  std::vector<std::vector<std::vector<int>>> frame_grid_;
};

}  // vslam

#endif  // VSLAM__FRAME_HPP_
