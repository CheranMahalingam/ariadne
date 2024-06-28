#ifndef VSLAM__MAP_POINT_HPP_
#define VSLAM__MAP_POINT_HPP_

#include <opencv2/core.hpp>

#include <map>

namespace vslam
{

class MapPoint
{
public:
  MapPoint(const cv::Mat & pos);

private:
  void computeDistinctDescriptors();
  void updateNormalAndDepth();

  cv::Mat world_pos_;
  cv::Mat normal_vector_;
};

}  // vslam

#endif  // VSLAM__MAP_POINT_HPP_
