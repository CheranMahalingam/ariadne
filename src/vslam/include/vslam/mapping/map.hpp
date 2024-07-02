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

private:
  std::set<std::shared_ptr<KeyFrame>> visited_key_frames_;
  std::set<std::shared_ptr<MapPoint>> visited_map_points_;
  std::vector<std::shared_ptr<MapPoint>> map_points_ref_;
};

}  // vslam

#endif  // VSLAM__MAP_HPP_
