#include "vslam/mapping/map.hpp"
#include "vslam/mapping/map_point.hpp"
#include "vslam/tracking/key_frame.hpp"
#include "vslam/utils.hpp"

#include <ranges>

namespace vslam
{

long int MapPoint::point_id = 0;

MapPoint::MapPoint(
  cv::Mat pos, std::shared_ptr<KeyFrame> key_frame, std::shared_ptr<Map> map)
: initial_kf_id(key_frame->curr_id),
  curr_id(point_id),
  world_pos_(pos),
  culled_(false),
  num_observations_(0),
  kf_ref_(key_frame),
  map_(map)
{
  point_id++;
}

void MapPoint::AddObservation(std::shared_ptr<KeyFrame> kf, int idx)
{
  if (observations_.find(kf) != observations_.end()) {
    return;
  }
  if (kf->stereo_key_points[idx] >= 0) {
    num_observations_ += 2;
  } else {
    num_observations_++;
  }
}

void MapPoint::EraseObservation(std::shared_ptr<KeyFrame> kf)
{
  if (observations_.find(kf) == observations_.end()) {
    return;
  }

  int point_idx = observations_.at(kf);
  if (kf_ref_->stereo_key_points[point_idx] >= 0) {
    num_observations_ -= 2;
  } else {
    num_observations_--;
  }

  observations_.erase(kf);
  if (kf == kf_ref_) {
    kf_ref_ = observations_.begin()->first;
  }
  if (num_observations_ <= 2) {
    Cull();
  }
}

void MapPoint::UpdateObservations()
{
  computeDistinctDescriptors();
  updateNormalAndDepth();
}

void MapPoint::Replace(std::shared_ptr<MapPoint> mp)
{
  culled_ = true;
  for (auto [kf, idx]:observations_) {
    if (mp->InKeyFrame(kf)) {
      kf->EraseMapPoint(idx);
    } else {
      // Place new map point at index of current map point.
      kf->AddMapPoint(mp, idx);
      mp->AddObservation(kf, idx);
    }
  }

  mp->IncreaseFound(num_found_);
  mp->IncreaseVisible(num_visible_);
  mp->UpdateObservations();

  observations_.clear();
  num_observations_ = 0;
  num_found_ = 0;
  num_visible_ = 0;
  map_->EraseMapPoint(shared_from_this());
}

void MapPoint::Cull()
{
  culled_ = true;
  for (auto & [kf, idx]:observations_) {
    kf->EraseMapPoint(idx);
  }
  observations_.clear();
  num_observations_ = 0;
  map_->EraseMapPoint(shared_from_this());
}

bool MapPoint::Culled() const
{
  return culled_;
}

bool MapPoint::InKeyFrame(std::shared_ptr<KeyFrame> kf) const
{
  return observations_.find(kf) != observations_.end();
}

void MapPoint::IncreaseFound(int n)
{
  num_found_ += n;
}

void MapPoint::IncreaseVisible(int n)
{
  num_visible_ += n;
}

float MapPoint::FoundRatio() const
{
  return num_found_ / num_visible_;
}

void MapPoint::SetWorldPos(cv::Mat world_pos)
{
  world_pos.copyTo(world_pos_);
}

cv::Mat MapPoint::GetWorldPos() const
{
  return world_pos_.clone();
}

cv::Mat MapPoint::GetNormal() const
{
  return normal_vector_.clone();
}

cv::Mat MapPoint::GetDescriptor() const
{
  return descriptor_.clone();
}

std::pair<float, float> MapPoint::GetDistanceInvariance() const
{
  return std::make_pair(min_distance_, max_distance_);
}

int MapPoint::GetNumObservations() const
{
  return num_observations_;
}

int MapPoint::GetObservationIndex(std::shared_ptr<KeyFrame> kf) const
{
  if (observations_.find(kf) == observations_.end()) {
    return -1;
  }
  return observations_.at(kf);
}

const std::map<std::shared_ptr<KeyFrame>, int> MapPoint::GetObservations() const
{
  return observations_;
}

std::shared_ptr<KeyFrame> MapPoint::GetReferenceKeyFrame() const
{
  return kf_ref_;
}

int MapPoint::PredictScale(
  float dist, float max_dist, float scale_factor, int num_scales)
{
  float ratio = max_dist / dist;
  int level = std::ceil(std::log(ratio) / std::log(scale_factor));
  if (level < 0) {
    level = 0;
  }
  if (level >= num_scales) {
    level = num_scales - 1;
  }
  return level;
}

void MapPoint::computeDistinctDescriptors()
{
  std::vector<cv::Mat> observed_descriptors;
  for (auto & [kf, idx]:observations_) {
    if (!kf->Culled()) {
      observed_descriptors.push_back(kf->descriptors[idx]);
    }
  }
  if (observed_descriptors.empty()) {
    return;
  }

  auto size = observed_descriptors.size();
  auto dist_matrix = std::vector<std::vector<int>>(
    size, std::vector<int>(size, 0));
  for (int i = 0; i < int(size); i++) {
    for (int j = i + 1; j < int(size); j++) {
      int dist = hamming_distance(
        observed_descriptors[i], observed_descriptors[j]);
      dist_matrix[i][j] = dist;
      dist_matrix[j][i] = dist;
    }
  }

  int min_median_dist = INT_MAX;
  cv::Mat min_descriptor;
  for (int i = 0; i < int(size); i++) {
    std::ranges::nth_element(dist_matrix[i], dist_matrix[i].begin() + size / 2);
    auto median = dist_matrix[i][size / 2];
    if (median < min_median_dist) {
      min_median_dist = median;
      min_descriptor = observed_descriptors[i];
    }
  }
  descriptor_ = min_descriptor;
}

void MapPoint::updateNormalAndDepth()
{
  auto normal = cv::Mat::zeros(3, 1, CV_32F);
  for (auto [kf, _]:observations_) {
    auto kf_camera_pos = kf->GetCameraCenter();
    auto normali = world_pos_ - kf_camera_pos;
    normal += normali / cv::norm(normali);
  }

  auto camera_point_vec = world_pos_ - kf_ref_->GetCameraCenter();
  auto dist = cv::norm(camera_point_vec);
  auto level = kf_ref_->key_points[observations_[kf_ref_]].octave;
  auto scale_factor = kf_ref_->image_scale_factors[level];
  auto num_levels = kf_ref_->image_scale_factors.size();

  max_distance_ = dist * scale_factor;
  min_distance_ = max_distance_ / kf_ref_->image_scale_factors[num_levels - 1];
  normal_vector_ = normal / observations_.size();
}

}  // vslam
