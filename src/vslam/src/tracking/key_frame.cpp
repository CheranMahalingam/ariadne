#include "vslam/mapping/map.hpp"
#include "vslam/mapping/map_point.hpp"
#include "vslam/tracking/key_frame.hpp"

#include <ranges>

namespace vslam
{

long int KeyFrame::key_frame_id = 0;

KeyFrame::KeyFrame(const Frame & frame, std::shared_ptr<Map> map)
: FrameBase::FrameBase(frame),
  curr_id(key_frame_id),
  frame_id(frame.curr_id),
  pose_(frame.pose),
  culled_(false),
  map_points_(frame.map_points),
  map_(map)
{
  key_frame_id++;
}

void KeyFrame::AddMapPoint(std::shared_ptr<MapPoint> point, int idx)
{
  std::lock_guard<std::mutex> lock(kf_mutex_);
  map_points_[idx] = point;
}

void KeyFrame::EraseMapPoint(int idx)
{
  std::lock_guard<std::mutex> lock(kf_mutex_);
  map_points_[idx] = nullptr;
}

void KeyFrame::AddConnection(std::shared_ptr<KeyFrame> kf, int weight)
{
  std::lock_guard<std::mutex> lock(kf_mutex_);
  connection_weights_[kf] = weight;
  updateCovisibilityGraph();
}

void KeyFrame::EraseConnection(std::shared_ptr<KeyFrame> kf)
{
  std::lock_guard<std::mutex> lock(kf_mutex_);
  if (connection_weights_.find(kf) == connection_weights_.end()) {
    return;
  }
  connection_weights_.erase(kf);
  updateCovisibilityGraph();
}


void KeyFrame::UpdateConnections()
{
  std::vector<std::shared_ptr<MapPoint>> curr_map_points;
  {
    std::lock_guard<std::mutex> lock(kf_mutex_);
    curr_map_points = map_points_;
  }

  std::map<std::shared_ptr<KeyFrame>, int> new_connection_weights;
  for (auto map_point:curr_map_points) {
    if (map_point == nullptr || map_point->Culled()) {
      continue;
    }

    auto observations = map_point->GetObservations();
    for (const auto & [kf, idx]:observations) {
      if (kf->curr_id == curr_id) {
        continue;
      }
      new_connection_weights[kf]++;
    }
  }
  if (new_connection_weights.empty()) {
    return;
  }

  int max_weight = 0;
  std::shared_ptr<KeyFrame> max_weight_kf = nullptr;
  std::vector<std::pair<int, std::shared_ptr<KeyFrame>>> connections;
  for (auto & [kf, weight]:new_connection_weights) {
    if (weight > max_weight) {
      max_weight = weight;
      max_weight_kf = kf;
    }
    if (weight >= MIN_KEY_FRAME_CONNECTION_WEIGHT) {
      connections.push_back(std::make_pair(weight, kf));
      kf->AddConnection(shared_from_this(), weight);
    }
  }
  if (connections.empty()) {
    connections.push_back(std::make_pair(max_weight, max_weight_kf));
    max_weight_kf->AddConnection(shared_from_this(), max_weight);
  }
  std::ranges::sort(connections, std::ranges::greater());

  std::vector<int> temp_ordered_weights;
  std::vector<std::shared_ptr<KeyFrame>> temp_ordered_connections;
  for (auto & [weight, kf]:connections) {
    temp_ordered_weights.push_back(weight);
    temp_ordered_connections.push_back(kf);
  }

  {
    std::lock_guard<std::mutex> lock(kf_mutex_);
    ordered_weights_ = temp_ordered_weights;
    ordered_connections_ = temp_ordered_connections;
    connection_weights_ = new_connection_weights;
    if (sp_tree_parent_ == nullptr && curr_id != 0) {
      sp_tree_parent_ = ordered_connections_.front();
      sp_tree_parent_->AddChild(shared_from_this());
    }
  }
}

void KeyFrame::AddChild(std::shared_ptr<KeyFrame> kf)
{
  std::lock_guard<std::mutex> lock(kf_mutex_);
  sp_tree_children_.insert(kf);
}

void KeyFrame::EraseChild(std::shared_ptr<KeyFrame> kf)
{
  std::lock_guard<std::mutex> lock(kf_mutex_);
  sp_tree_children_.erase(kf);
}

void KeyFrame::ChangeParent(std::shared_ptr<KeyFrame> kf)
{
  std::lock_guard<std::mutex> lock(kf_mutex_);
  sp_tree_parent_ = kf;
  kf->AddChild(shared_from_this());
}

void KeyFrame::AddLoopEdge(std::shared_ptr<KeyFrame> kf)
{
  std::lock_guard<std::mutex> lock(kf_mutex_);
  sp_tree_loop_edges_.insert(kf);
}

void KeyFrame::Cull()
{
  {
    std::lock_guard<std::mutex> lock(kf_mutex_);
    culled_ = true;
  }

  for (auto & [kf, weight]:connection_weights_) {
    kf->EraseConnection(shared_from_this());
  }

  {
    std::lock_guard<std::mutex> lock(kf_mutex_);
    for (auto & mp:map_points_) {
      if (mp != nullptr) {
        mp->EraseObservation(shared_from_this());
        mp->UpdateObservations();
      }
    }
  }

  {
    std::lock_guard<std::mutex> lock(kf_mutex_);

    ordered_weights_.clear();
    ordered_connections_.clear();
    connection_weights_.clear();

    std::vector<std::shared_ptr<KeyFrame>> candidates;
    candidates.push_back(sp_tree_parent_);
    while (!sp_tree_children_.empty()) {
      bool end_search = true;
      int max_weight = -1;
      std::shared_ptr<KeyFrame> new_child = nullptr;
      std::shared_ptr<KeyFrame> new_parent = nullptr;

      for (auto & child:sp_tree_children_) {
        if (child->Culled()) {
          continue;
        }

        auto child_connections = child->GetCovisibleKeyFrames();
        for (auto & connection:child_connections) {
          for (auto & candidate:candidates) {
            if (candidate->curr_id == connection->curr_id) {
              int weight = child->GetWeight(connection);
              if (weight > max_weight) {
                max_weight = weight;
                new_child = child;
                new_parent = connection;
                end_search = false;
              }
            }
          }
        }
      }

      if (end_search) {
        break;
      }
      new_child->ChangeParent(new_parent);
      candidates.push_back(new_child);
      sp_tree_children_.erase(new_child);
    }

    for (auto & kf:sp_tree_children_) {
      kf->ChangeParent(sp_tree_parent_);
    }
    sp_tree_parent_->EraseChild(shared_from_this());
    transform_cp = pose_.transform_cw * sp_tree_parent_->GetPoseInverse();
  }

  map_->EraseKeyFrame(shared_from_this());
}

bool KeyFrame::Culled()
{
  std::lock_guard<std::mutex> lock(kf_mutex_);
  return culled_;
}

cv::Mat KeyFrame::UnprojectToWorldFrame(int point_idx)
{
  std::lock_guard<std::mutex> lock(kf_mutex_);
  return unprojectKeyPoint(point_idx, pose_);
}

void KeyFrame::SetPose(cv::Mat pose_cw)
{
  std::lock_guard<std::mutex> lock(kf_mutex_);
  pose_.SetPose(pose_cw.clone());
}

cv::Mat KeyFrame::GetPose()
{
  std::lock_guard<std::mutex> lock(kf_mutex_);
  return pose_.transform_cw.clone();
}

cv::Mat KeyFrame::GetPoseInverse()
{
  std::lock_guard<std::mutex> lock(kf_mutex_);
  return pose_.transform_wc.clone();
}

cv::Mat KeyFrame::GetCameraCenter()
{
  std::lock_guard<std::mutex> lock(kf_mutex_);
  return pose_.translation_wc.clone();
}

cv::Mat KeyFrame::GetRotation()
{
  std::lock_guard<std::mutex> lock(kf_mutex_);
  return pose_.rotation_cw.clone();
}

cv::Mat KeyFrame::GetTranslation()
{
  std::lock_guard<std::mutex> lock(kf_mutex_);
  return pose_.translation_cw.clone();
}

int KeyFrame::GetWeight(std::shared_ptr<KeyFrame> kf)
{
  std::lock_guard<std::mutex> lock(kf_mutex_);
  if (connection_weights_.find(kf) == connection_weights_.end()) {
    return 0;
  }
  return connection_weights_.at(kf);
}

std::vector<std::shared_ptr<KeyFrame>> KeyFrame::GetCovisibleKeyFrames(
  int num_frames)
{
  std::lock_guard<std::mutex> lock(kf_mutex_);
  if (num_frames == 0 || num_frames >= int(ordered_connections_.size())) {
    return ordered_connections_;
  }
  return std::vector<std::shared_ptr<KeyFrame>>(
    ordered_connections_.begin(), ordered_connections_.begin() + num_frames);
}

std::shared_ptr<KeyFrame> KeyFrame::GetParent()
{
  std::lock_guard<std::mutex> lock(kf_mutex_);
  return sp_tree_parent_;
}

std::set<std::shared_ptr<KeyFrame>> KeyFrame::GetChildren()
{
  std::lock_guard<std::mutex> lock(kf_mutex_);
  return sp_tree_children_;
}

std::set<std::shared_ptr<KeyFrame>> KeyFrame::GetLoopEdges()
{
  std::lock_guard<std::mutex> lock(kf_mutex_);
  return sp_tree_loop_edges_;
}

std::shared_ptr<MapPoint> KeyFrame::GetMapPoint(int idx)
{
  std::lock_guard<std::mutex> lock(kf_mutex_);
  return map_points_[idx];
}

const std::vector<std::shared_ptr<MapPoint>> & KeyFrame::GetMapPoints()
{
  std::lock_guard<std::mutex> lock(kf_mutex_);
  return map_points_;
}

int KeyFrame::GetNumTrackedPoints(int min_observations)
{
  std::lock_guard<std::mutex> lock(kf_mutex_);
  int tracked_points = 0;
  for (auto mp:map_points_) {
    if (mp == nullptr || mp->Culled() ||
      mp->GetNumObservations() < min_observations)
    {
      continue;
    }
    tracked_points++;
  }
  return tracked_points;
}

void KeyFrame::updateCovisibilityGraph()
{
  std::vector<std::pair<int, std::shared_ptr<KeyFrame>>> new_connections;
  for (auto & [kf, weight]:connection_weights_) {
    new_connections.push_back(std::make_pair(weight, kf));
  }
  std::ranges::sort(new_connections, std::ranges::greater());

  std::vector<int> temp_ordered_weights;
  std::vector<std::shared_ptr<KeyFrame>> temp_ordered_connections;
  for (auto & [weight, kf]:new_connections) {
    temp_ordered_weights.push_back(weight);
    temp_ordered_connections.push_back(kf);
  }

  ordered_weights_ = temp_ordered_weights;
  ordered_connections_ = temp_ordered_connections;
}

}  // vslam
