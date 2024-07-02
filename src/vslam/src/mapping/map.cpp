#include "vslam/mapping/map.hpp"
#include "vslam/mapping/map_point.hpp"
#include "vslam/tracking/key_frame.hpp"

namespace vslam
{

Map::Map()
{
}

void Map::AddKeyFrame(std::shared_ptr<KeyFrame> kf)
{
  visited_key_frames_.insert(kf);
}

void Map::AddMapPoint(std::shared_ptr<MapPoint> mp)
{
  visited_map_points_.insert(mp);
}

}  // vslam
