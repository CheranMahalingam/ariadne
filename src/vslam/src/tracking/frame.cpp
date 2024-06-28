#include "vslam/tracking/frame.hpp"

namespace vslam
{

long int Frame::frame_id = 0;

Frame::Frame(
  const cv::Mat & grey, const cv::Mat & depth, rclcpp::Time timestamp,
  const std::vector<cv::KeyPoint> & key_points, const cv::Mat & descriptors,
  const DBoW3::Vocabulary & vocabulary,
  const CameraParams & camera_params)
: curr_id(frame_id),
  key_points_(key_points),
  stereo_key_points_(std::vector<float>(key_points.size())),
  key_point_depth_(std::vector<float>(key_points.size())),
  vocabulary_(vocabulary),
  camera_params_(camera_params)
{
  frame_id++;

  descriptors_ = std::vector<cv::Mat>(key_points.size());
  for (int i = 0; i < int(key_points.size()); i++) {
    descriptors_.push_back(descriptors.row(i));
  }

  // TODO: undistort key points using camera matrix.

  computeStereo(depth);

  frame_grid_ = std::vector<std::vector<std::vector<int>>>(
    camera_params_.width / 10, std::vector<std::vector<int>>(camera_params_.height / 10));
  populateGrid();
}

void Frame::ComputeBoW()
{
  vocabulary_.transform(descriptors_, bow_vector_, feat_vector_, BOW_LEVELS);
}

cv::Mat Frame::UnprojectToWorldFrame(int point_idx) const
{
  float z = key_point_depth_[point_idx];
  float u = key_points_[point_idx].pt.x;
  float v = key_points_[point_idx].pt.y;
  float x = (u - camera_params_.cx) * z / camera_params_.fx;
  float y = (v - camera_params_.cy) * z / camera_params_.fy;
  cv::Mat camera_pos = (cv::Mat_<float>(3, 1) << x, y, z);
  return camera_world_rotation_.t() * camera_pos + world_pos_;
}

void Frame::SetPose(cv::Mat pose)
{
  camera_world_transform_ = pose;
  camera_world_rotation_ = camera_world_transform_.rowRange(0, 3).colRange(0, 3);
  camera_world_translation_ = camera_world_transform_.rowRange(0, 3).col(3);
  world_pos_ = -camera_world_rotation_.t() * camera_world_translation_;
}

int Frame::GetSize() const
{
  return int(key_points_.size());
}

const DBoW3::FeatureVector & Frame::GetFeatures() const
{
  return feat_vector_;
}

const cv::Mat & Frame::GetDescriptor(int idx) const
{
  return descriptors_[idx];
}

const cv::KeyPoint & Frame::GetPoint(int idx) const
{
  return key_points_[idx];
}

void Frame::populateGrid()
{
  for (int i = 0; i < int(key_points_.size()); i++) {
    const auto & kp = key_points_[i];
    int pos_x = kp.pt.x / 10;
    int pos_y = kp.pt.y / 10;
    assert(pos_x < frame_grid_.size() && pos_y < frame_grid_[0].size());
    frame_grid_[pos_x][pos_y].push_back(i);
  }
}

void Frame::computeStereo(const cv::Mat & depth)
{
  for (int i = 0; i < int(key_points_.size()); i++) {
    auto kp = key_points_[i];
    float d = depth.at<float>(kp.pt.y, kp.pt.x);
    assert(d > 0);
    key_point_depth_[i] = d;
    float fx_mm = camera_params_.fx / 1000;
    stereo_key_points_[i] = kp.pt.x - fx_mm * camera_params_.depth_baseline / d;
  }
}

}  // vslam
