#ifndef VSLAM__KEY_FRAME_HPP_
#define VSLAM__KEY_FRAME_HPP_

#include "vslam/tracking/frame.hpp"
#include "vslam/utils.hpp"

#include "DBoW3/DBoW3.h"

#include <map>
#include <memory>
#include <set>

namespace vslam
{

class Map;
class MapPoint;

class KeyFrame : public FrameBase, std::enable_shared_from_this<KeyFrame>
{
public:
  static constexpr int MIN_KEY_FRAME_CONNECTION_WEIGHT = 15;

  static long int key_frame_id;

  KeyFrame(const Frame & frame, std::shared_ptr<Map> map);

  void AddMapPoint(std::shared_ptr<MapPoint> point, int idx);
  void EraseMapPoint(int idx);

  void AddConnection(std::shared_ptr<KeyFrame> kf, int weight);
  void EraseConnection(std::shared_ptr<KeyFrame> kf);
  void UpdateConnections();

  void AddChild(std::shared_ptr<KeyFrame> kf);
  void EraseChild(std::shared_ptr<KeyFrame> kf);
  void ChangeParent(std::shared_ptr<KeyFrame> kf);
  void AddLoopEdge(std::shared_ptr<KeyFrame> kf);

  void Cull();
  bool Culled() const;

  cv::Mat UnprojectToWorldFrame(int point_idx) const override;

  void SetPose(cv::Mat pose_cw);

  cv::Mat GetPose() const;
  cv::Mat GetPoseInverse() const;
  cv::Mat GetCameraCenter() const;
  cv::Mat GetRotation() const;
  cv::Mat GetTranslation() const;

  int GetWeight(std::shared_ptr<KeyFrame> kf) const;
  std::vector<std::shared_ptr<KeyFrame>> GetCovisibleKeyFrames(
    int num_frames = 0) const;

  std::shared_ptr<KeyFrame> GetParent() const;
  std::set<std::shared_ptr<KeyFrame>> GetChildren() const;
  std::set<std::shared_ptr<KeyFrame>> GetLoopEdges() const;

  std::shared_ptr<MapPoint> GetMapPoint(int idx) const;
  const std::vector<std::shared_ptr<MapPoint>> & GetMapPoints() const;
  int GetNumTrackedPoints(int min_observations) const;

  long int curr_id;
  long int frame_id;

  // Euclidean transform from parent key frame camera to current frame.
  cv::Mat transform_cp;

private:
  void updateCovisibilityGraph();

  Pose pose_;

  bool culled_;

  // Covisibility graph data structures
  std::vector<int> ordered_weights_;
  std::vector<std::shared_ptr<KeyFrame>> ordered_connections_;
  std::map<std::shared_ptr<KeyFrame>, int> connection_weights_;

  // Spanning tree data structures
  std::shared_ptr<KeyFrame> sp_tree_parent_;
  std::set<std::shared_ptr<KeyFrame>> sp_tree_children_;
  std::set<std::shared_ptr<KeyFrame>> sp_tree_loop_edges_;

  std::vector<std::shared_ptr<MapPoint>> map_points_;
  std::shared_ptr<Map> map_;
};

}  // vslam

#endif  // VSLAM__KEY_FRAME_HPP_
