#ifndef VSLAM__MAP_POINT_HPP_
#define VSLAM__MAP_POINT_HPP_

#include <opencv2/core.hpp>

#include <map>
#include <memory>

namespace vslam
{

class KeyFrame;
class Map;

class MapPoint
{
public:
  MapPoint(
    cv::Mat pos, std::shared_ptr<KeyFrame> key_frame, std::shared_ptr<Map> map);

  cv::Mat GetWorldPos() const;
  cv::Mat GetDescriptor() const;

private:
  void computeDistinctDescriptors();
  void updateNormalAndDepth();

  cv::Mat world_pos_;
  cv::Mat normal_vector_;

  cv::Mat descriptor_;

  std::shared_ptr<KeyFrame> kf_ref_;
  std::shared_ptr<Map> map_;
};

}  // vslam

#endif  // VSLAM__MAP_POINT_HPP_
