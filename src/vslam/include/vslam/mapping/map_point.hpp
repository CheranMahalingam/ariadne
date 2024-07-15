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
  static long int point_id;

  MapPoint(
    cv::Mat pos, std::shared_ptr<KeyFrame> key_frame, std::shared_ptr<Map> map);

  void AddObservation(std::shared_ptr<KeyFrame> kf, int idx);
  void EraseObservation(std::shared_ptr<KeyFrame> kf);
  void UpdateObservations();

  void Replace(std::shared_ptr<MapPoint> mp);

  void Cull();
  bool Culled() const;

  bool InKeyFrame(std::shared_ptr<KeyFrame> kf) const;

  void IncreaseFound(int n = 1);
  void IncreaseVisible(int n = 1);
  float FoundRatio() const;

  void SetWorldPos(cv::Mat world_pos);

  cv::Mat GetWorldPos() const;
  cv::Mat GetNormal() const;
  cv::Mat GetDescriptor() const;
  std::pair<float, float> GetDistanceInvariance() const;
  int GetNumObservations() const;
  int GetObservationIndex(std::shared_ptr<KeyFrame> kf) const;
  const std::map<std::shared_ptr<KeyFrame>, int> GetObservations() const;
  std::shared_ptr<KeyFrame> GetReferenceKeyFrame() const;

  static int PredictScale(
    float dist, float max_dist, float scale_factor, int num_scales);

  long int initial_kf_id;
  long int curr_id;

  bool tracked;
  float projected_x;
  float projected_y;
  float projected_right;
  int scale_level;
  float cos_viewing_angle;

private:
  /**
   * A map point may appear in multiple key frames after fusing points during
   * local mapping. Each observation may yield a slightly different descriptor,
   * so we replace them with the median of all descriptors.
   */
  void computeDistinctDescriptors();
  void updateNormalAndDepth();

  cv::Mat world_pos_;
  cv::Mat normal_vector_;
  cv::Mat descriptor_;

  float min_distance_;
  float max_distance_;

  bool culled_;

  int num_observations_;
  std::map<std::shared_ptr<KeyFrame>, int> observations_;

  std::shared_ptr<KeyFrame> kf_ref_;
  std::shared_ptr<Map> map_;

  int num_found_;
  int num_visible_;
};

}  // vslam

#endif  // VSLAM__MAP_POINT_HPP_
