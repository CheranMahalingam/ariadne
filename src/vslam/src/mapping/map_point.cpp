#include "vslam/mapping/map.hpp"
#include "vslam/mapping/map_point.hpp"
#include "vslam/tracking/key_frame.hpp"

namespace vslam
{

MapPoint::MapPoint(
  cv::Mat pos, std::shared_ptr<KeyFrame> key_frame, std::shared_ptr<Map> map)
: world_pos_(pos),
  culled_(false),
  num_observations_(0),
  kf_ref_(key_frame),
  map_(map)
{
  computeDistinctDescriptors();
  updateNormalAndDepth();
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

void MapPoint::Cull()
{
  culled_ = true;
  for (auto & [kf, idx]:observations_) {
    kf->EraseMapPoint(idx);
  }
  observations_.clear();
  map_->EraseMapPoint(shared_from_this());
}

bool MapPoint::Culled() const
{
  return culled_;
}

cv::Mat MapPoint::GetWorldPos() const
{
  return world_pos_.clone();
}

cv::Mat MapPoint::GetDescriptor() const
{
  return descriptor_.clone();
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

void MapPoint::computeDistinctDescriptors()
{
}

void MapPoint::updateNormalAndDepth()
{
  normal_vector_ = cv::Mat::zeros(3, 1, CV_32F);
}

}  // vslam
