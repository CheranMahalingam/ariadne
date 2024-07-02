#include "vslam/mapping/map.hpp"
#include "vslam/mapping/map_point.hpp"
#include "vslam/tracking/key_frame.hpp"

namespace vslam
{

long int KeyFrame::key_frame_id = 0;

KeyFrame::KeyFrame(const Frame & frame, std::shared_ptr<Map> map)
: curr_id(key_frame_id),
  frame_id(frame.curr_id),
  camera_params(frame.camera_params),
  key_points(frame.key_points),
  stereo_key_points(frame.stereo_key_points),
  depth_points(frame.depth_points),
  descriptors(frame.descriptors),
  bow_vector(frame.bow_vector),
  feat_vector(frame.feat_vector),
  vocabulary(frame.vocabulary),
  image_scale_factors(frame.image_scale_factors),
  frame_grid(frame.frame_grid),
  map_points_(frame.map_points),
  map_(map)
{
  key_frame_id++;
}

void KeyFrame::AddMapPoint(std::shared_ptr<MapPoint> point, int idx)
{
  map_points_[idx] = point;
}

void KeyFrame::SetPose(cv::Mat pose)
{
  camera_world_transform_ = pose;

  cv::Mat camera_world_rotation = camera_world_transform_.rowRange(0, 3).colRange(0, 3);
  cv::Mat camera_world_translation = camera_world_transform_.rowRange(0, 3).col(3);
  cv::Mat world_camera_rotation = camera_world_rotation.t();
  world_pos_ = -camera_world_rotation.t() * camera_world_translation;

  world_camera_transform_ = cv::Mat::eye(4, 4, camera_world_transform_.type());
  world_camera_rotation.copyTo(world_camera_transform_.rowRange(0, 3).colRange(0, 3));
  world_pos_.copyTo(world_camera_transform_.rowRange(0, 3).col(3));
}

cv::Mat KeyFrame::GetPose() const
{
  return camera_world_transform_.clone();
}

cv::Mat KeyFrame::GetPoseInverse() const
{
  return world_camera_transform_.clone();
}

cv::Mat KeyFrame::GetCameraCenter() const
{
  return world_pos_.clone();
}

const std::vector<std::shared_ptr<MapPoint>> & KeyFrame::GetMapPoints() const
{
  return map_points_;
}

}  // vslam
