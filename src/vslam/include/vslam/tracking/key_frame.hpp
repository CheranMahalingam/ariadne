#ifndef VSLAM__KEY_FRAME_HPP_
#define VSLAM__KEY_FRAME_HPP_

#include "vslam/tracking/frame.hpp"

#include "DBoW3/DBoW3.h"

#include <memory>

namespace vslam
{

class Map;
class MapPoint;

class KeyFrame
{
public:
  static long int key_frame_id;

  KeyFrame(const Frame & frame, std::shared_ptr<Map> map);

  void AddMapPoint(std::shared_ptr<MapPoint> point, int idx);

  void SetPose(cv::Mat pose);

  cv::Mat GetPose() const;
  cv::Mat GetPoseInverse() const;
  cv::Mat GetCameraCenter() const;
  const std::vector<std::shared_ptr<MapPoint>> & GetMapPoints() const;

  long int curr_id;
  long int frame_id;

  CameraParams camera_params;

  std::vector<cv::KeyPoint> key_points;
  std::vector<float> stereo_key_points;
  std::vector<float> depth_points;
  std::vector<cv::Mat> descriptors;

  cv::Mat camera_parent_transform;

  DBoW3::BowVector bow_vector;
  DBoW3::FeatureVector feat_vector;
  DBoW3::Vocabulary vocabulary;

  std::vector<float> image_scale_factors;

  std::vector<std::vector<std::vector<int>>> frame_grid;

private:
  cv::Mat camera_world_transform_;
  cv::Mat world_camera_transform_;
  cv::Mat world_pos_;

  std::vector<std::shared_ptr<MapPoint>> map_points_;
  std::shared_ptr<Map> map_;
};

}  // vslam

#endif  // VSLAM__KEY_FRAME_HPP_
