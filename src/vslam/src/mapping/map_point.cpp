#include "vslam/mapping/map.hpp"
#include "vslam/mapping/map_point.hpp"
#include "vslam/tracking/frame.hpp"
#include "vslam/tracking/key_frame.hpp"
#include "vslam/utils.hpp"

#include <mutex>
#include <ranges>

namespace vslam
{

long int MapPoint::point_id = 0;

MapPoint::MapPoint(
  cv::Mat pos, std::shared_ptr<KeyFrame> key_frame, std::shared_ptr<Map> map)
: initial_kf_id(key_frame->curr_id),
  world_pos_(pos.clone()),
  min_distance_(0),
  max_distance_(0),
  culled_(false),
  num_found_(1),
  num_visible_(1),
  num_observations_(0),
  kf_ref_(key_frame),
  map_(map)
{
  normal_vector_ = cv::Mat::zeros(3, 1, CV_32F);

  std::lock_guard<std::mutex> lock(map->mp_creation_mutex);
  curr_id = point_id;
  point_id++;
}

MapPoint::MapPoint(
  cv::Mat pos, std::shared_ptr<Map> map, std::shared_ptr<Frame> frame, int idx)
: initial_kf_id(-1),
  world_pos_(pos.clone()),
  culled_(false),
  num_found_(1),
  num_visible_(1),
  num_observations_(0),
  kf_ref_(nullptr),
  map_(map)
{
  auto normal = world_pos_ - frame->pose.translation_wc;
  auto dist = cv::norm(normal);
  normal_vector_ = normal / dist;

  auto level = frame->key_points[idx].octave;
  auto scale_factors = frame->image_scale_factors;
  max_distance_ = dist * scale_factors[level];
  min_distance_ = max_distance_ / scale_factors[scale_factors.size() - 1];

  frame->descriptors[idx].copyTo(descriptor_);

  std::lock_guard<std::mutex> lock(map->mp_creation_mutex);
  curr_id = point_id;
  point_id++;
}

void MapPoint::AddObservation(std::shared_ptr<KeyFrame> kf, int idx)
{
  std::lock_guard<std::mutex> lock(mp_mutex_);
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
  std::lock_guard<std::mutex> lock(mp_mutex_);
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
  if (mp->curr_id == this->curr_id) {
    return;
  }

  std::map<std::shared_ptr<KeyFrame>, int> observations;
  int num_found;
  int num_visible;
  {
    std::lock_guard<std::mutex> lock(mp_mutex_);
    culled_ = true;
    observations = observations_;
    observations_.clear();
    num_found = num_found_;
    num_visible = num_visible_;
    replaced_point_ = mp;
  }

  for (auto [kf, idx]:observations) {
    if (mp->InKeyFrame(kf)) {
      kf->EraseMapPoint(idx);
    } else {
      // Place new map point at index of current map point.
      kf->AddMapPoint(mp, idx);
      mp->AddObservation(kf, idx);
    }
  }

  mp->IncreaseFound(num_found);
  mp->IncreaseVisible(num_visible);
  mp->UpdateObservations();

  map_->EraseMapPoint(shared_from_this());
}

void MapPoint::Cull()
{
  std::map<std::shared_ptr<KeyFrame>, int> observations;
  {
    std::lock_guard<std::mutex> lock(mp_mutex_);
    culled_ = true;
    observations = observations_;

    observations_.clear();
    num_observations_ = 0;
  }

  for (auto & [kf, idx]:observations) {
    kf->EraseMapPoint(idx);
  }
  map_->EraseMapPoint(shared_from_this());
}

bool MapPoint::Culled()
{
  std::lock_guard<std::mutex> lock(mp_mutex_);
  return culled_;
}

bool MapPoint::InKeyFrame(std::shared_ptr<KeyFrame> kf)
{
  std::lock_guard<std::mutex> lock(mp_mutex_);
  return observations_.find(kf) != observations_.end();
}

void MapPoint::IncreaseFound(int n)
{
  std::lock_guard<std::mutex> lock(mp_mutex_);
  num_found_ += n;
}

void MapPoint::IncreaseVisible(int n)
{
  std::lock_guard<std::mutex> lock(mp_mutex_);
  num_visible_ += n;
}

float MapPoint::FoundRatio()
{
  std::lock_guard<std::mutex> lock(mp_mutex_);
  return num_found_ / num_visible_;
}

void MapPoint::SetWorldPos(cv::Mat world_pos)
{
  std::lock_guard<std::mutex> lock(mp_mutex_);
  world_pos.copyTo(world_pos_);
}

cv::Mat MapPoint::GetWorldPos()
{
  std::lock_guard<std::mutex> lock(mp_mutex_);
  return world_pos_.clone();
}

cv::Mat MapPoint::GetNormal()
{
  std::lock_guard<std::mutex> lock(mp_mutex_);
  return normal_vector_.clone();
}

cv::Mat MapPoint::GetDescriptor()
{
  std::lock_guard<std::mutex> lock(mp_mutex_);
  return descriptor_.clone();
}

std::pair<float, float> MapPoint::GetDistanceInvariance()
{
  std::lock_guard<std::mutex> lock(mp_mutex_);
  return std::make_pair(min_distance_, max_distance_);
}

std::optional<std::shared_ptr<MapPoint>> MapPoint::GetReplacedPoint()
{
  std::lock_guard<std::mutex> lock(mp_mutex_);
  if (replaced_point_ == nullptr) {
    return {};
  }
  return replaced_point_;
}

int MapPoint::GetNumObservations()
{
  std::lock_guard<std::mutex> lock(mp_mutex_);
  return num_observations_;
}

int MapPoint::GetObservationIndex(std::shared_ptr<KeyFrame> kf)
{
  std::lock_guard<std::mutex> lock(mp_mutex_);
  if (observations_.find(kf) == observations_.end()) {
    return -1;
  }
  return observations_.at(kf);
}

const std::map<std::shared_ptr<KeyFrame>, int> MapPoint::GetObservations()
{
  std::lock_guard<std::mutex> lock(mp_mutex_);
  return observations_;
}

std::shared_ptr<KeyFrame> MapPoint::GetReferenceKeyFrame()
{
  std::lock_guard<std::mutex> lock(mp_mutex_);
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
  std::map<std::shared_ptr<KeyFrame>, int> observations;
  {
    std::lock_guard<std::mutex> lock(mp_mutex_);
    observations = observations_;
  }

  std::vector<cv::Mat> observed_descriptors;
  for (auto & [kf, idx]:observations) {
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

  {
    std::lock_guard<std::mutex> lock(mp_mutex_);
    descriptor_ = min_descriptor;
  }
}

void MapPoint::updateNormalAndDepth()
{
  std::map<std::shared_ptr<KeyFrame>, int> observations;
  std::shared_ptr<KeyFrame> curr_kf;
  cv::Mat pos;
  {
    std::lock_guard<std::mutex> lock(mp_mutex_);
    observations = observations_;
    curr_kf = kf_ref_;
    pos = world_pos_.clone();
  }

  auto normal = cv::Mat::zeros(3, 1, CV_32F);
  for (auto [kf, _]:observations) {
    auto kf_camera_pos = kf->GetCameraCenter();
    auto normali = pos - kf_camera_pos;
    normal += normali / cv::norm(normali);
  }

  auto camera_point_vec = pos - curr_kf->GetCameraCenter();
  auto dist = cv::norm(camera_point_vec);
  auto level = curr_kf->key_points[observations_[curr_kf]].octave;
  auto scale_factor = curr_kf->image_scale_factors[level];
  auto num_levels = curr_kf->image_scale_factors.size();

  {
    std::lock_guard<std::mutex> lock(mp_mutex_);
    max_distance_ = dist * scale_factor;
    min_distance_ = max_distance_ / curr_kf->image_scale_factors[num_levels - 1];
    normal_vector_ = normal / observations.size();
  }
}

}  // vslam
