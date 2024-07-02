#include "vslam/mapping/map.hpp"
#include "vslam/mapping/map_point.hpp"
#include "vslam/tracking/key_frame.hpp"

namespace vslam
{

MapPoint::MapPoint(
  cv::Mat pos, std::shared_ptr<KeyFrame> key_frame, std::shared_ptr<Map> map)
: world_pos_(pos),
  kf_ref_(key_frame),
  map_(map)
{
  computeDistinctDescriptors();
  updateNormalAndDepth();
}

cv::Mat MapPoint::GetWorldPos() const
{
  return world_pos_.clone();
}

cv::Mat MapPoint::GetDescriptor() const
{
  return descriptor_.clone();
}

void MapPoint::computeDistinctDescriptors()
{
}

void MapPoint::updateNormalAndDepth()
{
  normal_vector_ = cv::Mat::zeros(3, 1, CV_32F);
}

}  // vslam
