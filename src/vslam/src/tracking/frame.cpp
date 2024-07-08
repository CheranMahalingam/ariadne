#include "vslam/mapping/map_point.hpp"
#include "vslam/tracking/feature_extractor.hpp"
#include "vslam/tracking/frame.hpp"
#include "vslam/tracking/key_frame.hpp"

#include <cmath>

namespace vslam
{

FrameBase::FrameBase(
  const cv::Mat & depth,
  const std::vector<cv::KeyPoint> & key_points,
  const cv::Mat & desc,
  const FeatureExtractor * extractor,
  const DBoW3::Vocabulary & vocabulary,
  const CameraParams & camera_params)
: camera_params(camera_params),
  key_points(key_points),
  stereo_key_points(std::vector<float>(key_points.size())),
  depth_points(std::vector<float>(key_points.size())),
  vocabulary(vocabulary)
{
  scale_factor = extractor->GetScaleFactor();
  image_scale_factors = extractor->GetScaleFactors();

  for (int i = 0; i < int(key_points.size()); i++) {
    descriptors.push_back(desc.row(i));
  }

  // TODO: Undistort key points using camera matrix.

  computeStereo(depth);

  grid = std::vector<std::vector<std::vector<int>>>(
    camera_params.width / FRAME_GRID_SIZE,
    std::vector<std::vector<int>>(camera_params.height / FRAME_GRID_SIZE));
  populateGrid();
}

FrameBase::FrameBase(const FrameBase & frame)
: camera_params(frame.camera_params),
  key_points(frame.key_points),
  stereo_key_points(frame.stereo_key_points),
  depth_points(frame.depth_points),
  descriptors(frame.descriptors),
  bow_vector(frame.bow_vector),
  feat_vector(frame.feat_vector),
  vocabulary(frame.vocabulary),
  scale_factor(frame.scale_factor),
  image_scale_factors(frame.image_scale_factors),
  grid(frame.grid)
{
}

FrameBase::~FrameBase() {}

void FrameBase::ComputeBoW()
{
  if (bow_vector.empty()) {
    vocabulary.transform(descriptors, bow_vector, feat_vector, BOW_LEVELS);
  }
}

std::vector<int> FrameBase::FindNeighbourFeatures(
  float x, float y, float radius, int min_level) const
{
  int max_level = image_scale_factors.size() - 1;
  return FindNeighbourFeatures(x, y, radius, min_level, max_level);
}

std::vector<int> FrameBase::FindNeighbourFeatures(
  float x, float y, float radius, int min_level, int max_level) const
{
  int x_min = std::max(0, int(std::floor((x - radius) / FRAME_GRID_SIZE)));
  int x_max = std::min(
    int(grid.size() - 1), int(std::ceil((x + radius) / FRAME_GRID_SIZE)));
  int y_min = std::max(0, int(std::floor((y - radius) / FRAME_GRID_SIZE)));
  int y_max = std::min(
    int(grid[0].size() - 1), int(std::ceil((y + radius) / FRAME_GRID_SIZE)));

  std::vector<int> neighbour_points;
  for (int x = x_min; x <= x_max; x++) {
    for (int y = y_min; y <= y_max; y++) {
      const auto & cell = grid[x][y];
      if (cell.empty()) {
        continue;
      }

      for (auto point_idx:cell) {
        const auto & point = key_points[point_idx];
        if (point.octave < min_level || point.octave > max_level) {
          continue;
        }

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

cv::Mat FrameBase::unprojectKeyPoint(int point_idx, Pose pose) const
{
  float z = depth_points[point_idx];
  if (z < 0) {
    return cv::Mat();
  }
  float u = key_points[point_idx].pt.x;
  float v = key_points[point_idx].pt.y;
  float x = (u - camera_params.cx) * z / camera_params.fx;
  float y = (v - camera_params.cy) * z / camera_params.fy;
  cv::Mat point_c = (cv::Mat_<float>(3, 1) << x, y, z);
  return pose.rotation_wc * point_c + pose.translation_wc;
}

void FrameBase::populateGrid()
{
  for (int i = 0; i < int(key_points.size()); i++) {
    const auto & kp = key_points[i];
    int pos_x = kp.pt.x / FRAME_GRID_SIZE;
    int pos_y = kp.pt.y / FRAME_GRID_SIZE;
    assert(pos_x < grid.size() && pos_y < grid[0].size());
    grid[pos_x][pos_y].push_back(i);
  }
}

void FrameBase::computeStereo(const cv::Mat & depth)
{
  for (int i = 0; i < int(key_points.size()); i++) {
    auto kp = key_points[i];
    float d = depth.at<float>(kp.pt.y, kp.pt.x);
    assert(d > 0);
    depth_points[i] = d;
    float bf = camera_params.fx * camera_params.depth_baseline / 1000.0;
    stereo_key_points[i] = kp.pt.x - bf / d;
  }
}

long int Frame::frame_id = 0;

Frame::Frame(const Frame & frame)
: FrameBase(frame),
  curr_id(frame_id),
  pose(frame.pose),
  kf_ref(frame.kf_ref),
  map_points(frame.map_points)
{
  frame_id++;
}

Frame::Frame(
  const cv::Mat & depth,
  rclcpp::Time timestamp,
  const std::vector<cv::KeyPoint> & key_points,
  const cv::Mat & descriptors,
  const FeatureExtractor * extractor,
  const DBoW3::Vocabulary & vocabulary,
  const CameraParams & camera_params)
: FrameBase(
    depth, key_points, descriptors, extractor, vocabulary, camera_params),
  curr_id(frame_id),
  map_points(std::vector<std::shared_ptr<MapPoint>>(key_points.size()))
{
  frame_id++;
}

cv::Mat Frame::UnprojectToWorldFrame(int point_idx) const
{
  return unprojectKeyPoint(point_idx, pose);
}

bool Frame::ValidFrustumProjection(std::shared_ptr<MapPoint> mp) const
{
  mp->tracked = false;

  // Find coordinates of point in world frame.
  cv::Mat mp_w = mp->GetWorldPos();
  // Find coordinates of point in camera frame.
  cv::Mat mp_c = pose.rotation_cw * mp_w + pose.translation_cw;
  float z = mp_c.at<float>(2);
  if (z < 0.0) {
    return false;
  }
  float u = camera_params.fx * mp_c.at<float>(0) / z + camera_params.cx;
  float v = camera_params.fy * mp_c.at<float>(1) / z + camera_params.cy;
  // TODO: Validate that point (u,v) lies within frame boundaries

  // Find translation connecting point and camera in world frame.
  cv::Mat pc_w = mp_w - pose.translation_wc;
  auto dist = cv::norm(pc_w);
  auto [min_dist, max_dist] = mp->GetDistanceInvariance();
  if (dist < min_dist || dist > max_dist) {
    return false;
  }

  cv::Mat normal = mp->GetNormal();
  float cos = pc_w.dot(normal) / dist;
  if (cos < FRUSTUM_COSINE_THRESHOLD) {
    return false;
  }

  mp->tracked = true;
  mp->projected_x = u;
  mp->projected_y = v;
  float bf = camera_params.fx * camera_params.depth_baseline / 1000.0;
  mp->projected_right = u - bf / z;
  auto predicted_level = MapPoint::PredictScale(
    dist, max_dist, scale_factor, image_scale_factors.size());
  mp->scale_level = predicted_level;
  mp->cos_viewing_angle = cos;

  return true;
}

void Frame::SetPose(cv::Mat pose_cw)
{
  pose.SetPose(pose_cw);
}

}  // vslam
