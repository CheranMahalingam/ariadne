#ifndef VSLAM__KEY_FRAME_HPP_
#define VSLAM__KEY_FRAME_HPP_

#include "vslam/tracking/frame.hpp"

#include "DBoW3/DBoW3.h"

#include <map>
#include <memory>
#include <set>

namespace vslam
{

class Map;
class MapPoint;

class KeyFrame : public std::enable_shared_from_this<KeyFrame>
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

  void SetPose(cv::Mat pose);

  cv::Mat GetPose() const;
  cv::Mat GetPoseInverse() const;
  cv::Mat GetCameraCenter() const;

  int GetWeight(std::shared_ptr<KeyFrame> kf) const;
  std::vector<std::shared_ptr<KeyFrame>> GetCovisibleKeyFrames(
    int num_frames = 0) const;

  std::shared_ptr<KeyFrame> GetParent() const;
  std::set<std::shared_ptr<KeyFrame>> GetChildren() const;
  std::set<std::shared_ptr<KeyFrame>> GetLoopEdges() const;

  const std::vector<std::shared_ptr<MapPoint>> & GetMapPoints() const;

  long int curr_id;
  long int frame_id;

  CameraParams camera_params;

  std::vector<cv::KeyPoint> key_points;
  std::vector<float> stereo_key_points;
  std::vector<float> depth_points;
  std::vector<cv::Mat> descriptors;

  cv::Mat camera_parent_transform;

  DBoW3::BowVector bow_vector;
  DBoW3::FeatureVector feat_vector;
  DBoW3::Vocabulary vocabulary;

  std::vector<float> image_scale_factors;

  std::vector<std::vector<std::vector<int>>> frame_grid;

private:
  void updateCovisibilityGraph();

  cv::Mat camera_world_transform_;
  cv::Mat world_camera_transform_;
  cv::Mat world_pos_;

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
