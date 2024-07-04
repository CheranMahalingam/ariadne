#ifndef VSLAM__MAP_POINT_HPP_
#define VSLAM__MAP_POINT_HPP_

#include <opencv2/core.hpp>

#include <map>
#include <memory>

namespace vslam
{

class KeyFrame;
class Map;

class MapPoint : public std::enable_shared_from_this<MapPoint>
{
public:
  MapPoint(
    cv::Mat pos, std::shared_ptr<KeyFrame> key_frame, std::shared_ptr<Map> map);

  void AddObservation(std::shared_ptr<KeyFrame> kf, int idx);
  void EraseObservation(std::shared_ptr<KeyFrame> kf);

  void Cull();
  bool Culled() const;

  cv::Mat GetWorldPos() const;
  cv::Mat GetDescriptor() const;
  int GetNumObservations() const;
  int GetObservationIndex(std::shared_ptr<KeyFrame> kf) const;
  const std::map<std::shared_ptr<KeyFrame>, int> GetObservations() const;
  std::shared_ptr<KeyFrame> GetReferenceKeyFrame() const;

private:
  void computeDistinctDescriptors();
  void updateNormalAndDepth();

  cv::Mat world_pos_;
  cv::Mat normal_vector_;
  cv::Mat descriptor_;

  bool culled_;

  int num_observations_;
  std::map<std::shared_ptr<KeyFrame>, int> observations_;

  std::shared_ptr<KeyFrame> kf_ref_;
  std::shared_ptr<Map> map_;
};

}  // vslam

#endif  // VSLAM__MAP_POINT_HPP_
