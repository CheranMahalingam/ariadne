#include "vslam/mapping/map_point.hpp"
#include "vslam/tracking/frame.hpp"
#include "vslam/tracking/key_frame.hpp"

#include <cmath>

namespace vslam
{

long int Frame::frame_id = 0;

Frame::Frame(const Frame & frame)
: curr_id(frame_id),
  camera_params(frame.camera_params),
  key_points(frame.key_points),
  stereo_key_points(frame.stereo_key_points),
  depth_points(frame.depth_points),
  descriptors(frame.descriptors),
  bow_vector(frame.bow_vector),
  feat_vector(frame.feat_vector),
  vocabulary(frame.vocabulary),
  image_scale_factors(frame.image_scale_factors),
  kf_ref(frame.kf_ref),
  map_points(frame.map_points),
  frame_grid(frame.frame_grid)
{
  frame_id++;

  SetPose(frame.camera_world_transform);
}

Frame::Frame(
  const cv::Mat & grey, const cv::Mat & depth, rclcpp::Time timestamp,
  const std::vector<cv::KeyPoint> & key_points, const cv::Mat & desc,
  const DBoW3::Vocabulary & vocabulary,
  const std::vector<float> & scale_factors,
  const CameraParams & camera_params)
: curr_id(frame_id),
  camera_params(camera_params),
  key_points(key_points),
  stereo_key_points(std::vector<float>(key_points.size())),
  depth_points(std::vector<float>(key_points.size())),
  descriptors(std::vector<cv::Mat>(key_points.size())),
  vocabulary(vocabulary),
  image_scale_factors(image_scale_factors),
  map_points(std::vector<std::shared_ptr<MapPoint>>(key_points.size()))
{
  frame_id++;

  for (int i = 0; i < int(key_points.size()); i++) {
    descriptors.push_back(desc.row(i));
  }

  // TODO: Undistort key points using camera matrix.

  computeStereo(depth);

  frame_grid = std::vector<std::vector<std::vector<int>>>(
    camera_params.width / FRAME_GRID_SIZE,
    std::vector<std::vector<int>>(camera_params.height / FRAME_GRID_SIZE));
  populateGrid();
}

void Frame::ComputeBoW()
{
  vocabulary.transform(descriptors, bow_vector, feat_vector, BOW_LEVELS);
}

cv::Mat Frame::UnprojectToWorldFrame(int point_idx) const
{
  float z = depth_points[point_idx];
  float u = key_points[point_idx].pt.x;
  float v = key_points[point_idx].pt.y;
  float x = (u - camera_params.cx) * z / camera_params.fx;
  float y = (v - camera_params.cy) * z / camera_params.fy;
  cv::Mat camera_pos = (cv::Mat_<float>(3, 1) << x, y, z);
  return camera_world_rotation.t() * camera_pos + world_pos;
}

std::vector<int> Frame::FindNeighbourFeatures(
  float x, float y, float radius) const
{
  int x_min = std::max(0, int(std::floor((x - radius) / FRAME_GRID_SIZE)));
  int x_max = std::min(
    int(frame_grid.size() - 1), int(std::ceil((x + radius) / FRAME_GRID_SIZE)));
  int y_min = std::max(0, int(std::floor((y - radius) / FRAME_GRID_SIZE)));
  int y_max = std::min(
    int(frame_grid[0].size() - 1), int(std::ceil((y + radius) / FRAME_GRID_SIZE)));

  std::vector<int> neighbour_points;
  for (int x = x_min; x <= x_max; x++) {
    for (int y = y_min; y <= y_max; y++) {
      const auto & cell = frame_grid[x][y];
      if (cell.empty()) {
        continue;
      }

      for (auto point_idx:cell) {
        const auto & point = key_points[point_idx];
        float x_dist = std::fabs(point.pt.x - x);
        float y_dist = std::fabs(point.pt.y - y);
        if (x_dist < radius && y_dist < radius) {
          neighbour_points.push_back(point_idx);
        }
      }
    }
  }
  return neighbour_points;
}

void Frame::SetPose(cv::Mat pose)
{
  camera_world_transform = pose;
  camera_world_rotation = camera_world_transform.rowRange(0, 3).colRange(0, 3);
  camera_world_translation = camera_world_transform.rowRange(0, 3).col(3);
  world_pos = -camera_world_rotation.t() * camera_world_translation;
}

void Frame::populateGrid()
{
  for (int i = 0; i < int(key_points.size()); i++) {
    const auto & kp = key_points[i];
    int pos_x = kp.pt.x / FRAME_GRID_SIZE;
    int pos_y = kp.pt.y / FRAME_GRID_SIZE;
    assert(pos_x < frame_grid.size() && pos_y < frame_grid[0].size());
    frame_grid[pos_x][pos_y].push_back(i);
  }
}

void Frame::computeStereo(const cv::Mat & depth)
{
  for (int i = 0; i < int(key_points.size()); i++) {
    auto kp = key_points[i];
    float d = depth.at<float>(kp.pt.y, kp.pt.x);
    assert(d > 0);
    depth_points[i] = d;
    float fx_mm = camera_params.fx / 1000;
    stereo_key_points[i] = kp.pt.x - fx_mm * camera_params.depth_baseline / d;
  }
}

}  // vslam
