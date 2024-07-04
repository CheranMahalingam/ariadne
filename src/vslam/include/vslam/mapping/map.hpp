#ifndef VSLAM__MAP_HPP_
#define VSLAM__MAP_HPP_

#include <memory>
#include <set>
#include <vector>

namespace vslam
{

class KeyFrame;
class MapPoint;

class Map
{
public:
  Map();

  void AddKeyFrame(std::shared_ptr<KeyFrame> kf);
  void AddMapPoint(std::shared_ptr<MapPoint> mp);
  void EraseKeyFrame(std::shared_ptr<KeyFrame> kf);
  void EraseMapPoint(std::shared_ptr<MapPoint> mp);

  void SetReferenceMapPoints(const std::vector<std::shared_ptr<MapPoint>> & map_points);

  int GetKeyFrameCount() const;
  int GetMapPointCount() const;
  std::vector<std::shared_ptr<KeyFrame>> GetKeyFrames() const;
  std::vector<std::shared_ptr<MapPoint>> GetMapPoints() const;

  std::shared_ptr<KeyFrame> origin;

private:
  std::set<std::shared_ptr<KeyFrame>> visited_key_frames_;
  std::set<std::shared_ptr<MapPoint>> visited_map_points_;
  std::vector<std::shared_ptr<MapPoint>> local_map_points_;
};

}  // vslam

#endif  // VSLAM__MAP_HPP_
