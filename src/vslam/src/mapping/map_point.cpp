#include "vslam/mapping/map_point.hpp"

namespace vslam
{

MapPoint::MapPoint(const cv::Mat & pos)
: world_pos_(pos)
{
  computeDistinctDescriptors();
  updateNormalAndDepth();
}

void MapPoint::computeDistinctDescriptors()
{
}

void MapPoint::updateNormalAndDepth()
{
  normal_vector_ = cv::Mat::zeros(3, 1, CV_32F);
}

}  // vslam
