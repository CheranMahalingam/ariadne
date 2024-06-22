#include "vslam/tracking/frame.hpp"

namespace vslam
{

long int Frame::frame_id = 0;

Frame::Frame(
  const cv::Mat & grey, const cv::Mat & depth, rclcpp::Time timestamp,
  const std::vector<cv::KeyPoint> & key_points, const cv::Mat & descriptors,
  const DBoW3::Vocabulary & vocabulary,
  const CameraParams & camera_params)
: key_points_(key_points),
  stereo_key_points_(std::vector<float>(key_points.size())),
  key_point_depth_(std::vector<float>(key_points.size())),
  vocabulary_(vocabulary),
  camera_params_(camera_params)
{
  curr_id_ = frame_id;
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
