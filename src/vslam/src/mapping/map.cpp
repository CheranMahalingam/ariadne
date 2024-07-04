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

void Map::EraseKeyFrame(std::shared_ptr<KeyFrame> kf)
{
  visited_key_frames_.erase(kf);
}

void Map::EraseMapPoint(std::shared_ptr<MapPoint> mp)
{
  visited_map_points_.erase(mp);
}

void Map::SetReferenceMapPoints(const std::vector<std::shared_ptr<MapPoint>> & map_points)
{
  local_map_points_ = map_points;
}

int Map::GetKeyFrameCount() const
{
  return visited_key_frames_.size();
}

int Map::GetMapPointCount() const
{
  return visited_map_points_.size();
}

std::vector<std::shared_ptr<KeyFrame>> Map::GetKeyFrames() const
{
  return std::vector<std::shared_ptr<KeyFrame>>(
    visited_key_frames_.begin(), visited_key_frames_.end());
}

std::vector<std::shared_ptr<MapPoint>> Map::GetMapPoints() const
{
  return std::vector<std::shared_ptr<MapPoint>>(
    visited_map_points_.begin(), visited_map_points_.end());
}

}  // vslam
