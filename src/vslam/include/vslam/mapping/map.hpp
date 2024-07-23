#ifndef VSLAM__MAP_HPP_
#define VSLAM__MAP_HPP_

#include <memory>
#include <mutex>
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

  int GetKeyFrameCount();
  int GetMapPointCount();
  std::vector<std::shared_ptr<KeyFrame>> GetKeyFrames();
  std::vector<std::shared_ptr<MapPoint>> GetMapPoints();

  std::mutex mp_creation_mutex;

  std::shared_ptr<KeyFrame> origin;

private:
  std::mutex map_mutex_;

  std::set<std::shared_ptr<KeyFrame>> visited_key_frames_;
  std::set<std::shared_ptr<MapPoint>> visited_map_points_;
  std::vector<std::shared_ptr<MapPoint>> local_map_points_;
};

}  // vslam

#endif  // VSLAM__MAP_HPP_
