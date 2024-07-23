#ifndef VSLAM__FEATURE_MATCHER_HPP_
#define VSLAM__FEATURE_MATCHER_HPP_

#include "vslam/tracking/frame.hpp"
#include "vslam/tracking/key_frame.hpp"

#include <opencv2/core.hpp>

#include <memory>
#include <unordered_map>
#include <vector>

namespace vslam
{

class MapPoint;

class FeatureMatcher
{
public:
  // Number of bins in histogram where points are placed according to rotation.
  static constexpr int HISTOGRAM_BINS = 30;
  // Number of bins in the rotation histogram that pass the consistency check.
  static constexpr int FILTERED_ORIENTATION_BINS = 5;
  // Minimum Hamming distance between descriptors for points to be considered
  // for a match.
  static constexpr int MIN_DISTANCE_THRESHOLD = 50;
  static constexpr float FUNDAMENTAL_MATRIX_OUTLIER_THRESHOLD = 3.84;
  static constexpr int MAX_DESCRIPTOR_DISTANCE = 256;

  FeatureMatcher(float nn_dist_ratio);

  /**
   * Finds matching points between key_frame and curr_frame using BoW feature
   * vectors. If there are features that belong to the same vocabulary tree
   * node, the point with the lowest Hamming distance is selected as a match.
   * Outliers are filtered using a rotation consistency check.
   */
  int BoWSearch(
    KeyFrame & key_frame, const Frame & curr_frame,
    std::vector<std::shared_ptr<MapPoint>> & matching_points) const;

  /**
   * Finds matching points between curr_frame and prev_frame by iterating over
   * each point in prev_frame and searching for all points in curr_frame within
   * radius, search_radius_threshold. Hamming distance is used to select the
   * best match and outliers are filtered using a rotation consistency check.
   */
  int ProjectionSearch(
    Frame & curr_frame, const Frame & prev_frame,
    int search_radius_threshold) const;
  /**
   * Finds matching points between curr_frame and map points (from local map)
   * by iterating over each map point and searching for points in curr_frame
   * within radius, search_radius_threshold. Hamming distance is used to select
   * the best match and the points' pyramid levels are used as a sanity check.
   */
  int ProjectionSearch(
    Frame & curr_frame, const std::vector<std::shared_ptr<MapPoint>> & map_points,
    int search_radius_threshold = 3) const;

  /**
   * Finds matching points between two key frames using BoW vectors. Features
   * belonging to the same vocabulary tree node are compared using Hamming
   * distance to select a match. Only points that satisfy the epipolar geometry
   * constraint are considered and additional outliers are filtered using a
   * rotation consistency check.
   */
  int EpipolarSearch(
    KeyFrame & kf1, KeyFrame & kf2, const cv::Mat f_matrix,
    std::vector<std::pair<int, int>> & matching_pairs) const;

  /**
   * Fuses map points with points observed in a key frame. Map points are
   * projected onto the pixel frame and points within the search radius are
   * compared using Hamming distance for association.
   */
  int Fuse(
    std::shared_ptr<KeyFrame> kf,
    std::vector<std::shared_ptr<MapPoint>> & map_points,
    int search_radius_threshold = 3) const;

private:
  /**
   * Computes the number of matching points by filtering using a rotation
   * histogram. The histogram holds the number of points detected with angles
   * belonging to different ranges. Only points belonging to ranges with the
   * most points are kept.
   */
  int applyHistogramFilter(
    const std::array<std::vector<int>, HISTOGRAM_BINS> & rotation_histogram,
    std::vector<bool> & valid_matches) const;

  bool validateEpipolarConstraints(
    const cv::KeyPoint & kp1, const cv::KeyPoint & kp2, const cv::Mat f_matrix,
    const KeyFrame & kf) const;

  float nn_dist_ratio_;
};

}  // vslam

#endif  // VSLAM__FEATURE_MATCHER_HPP_
