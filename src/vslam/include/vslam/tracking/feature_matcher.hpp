#ifndef VSLAM__FEATURE_MATCHER_HPP_
#define VSLAM__FEATURE_MATCHER_HPP_

#include "vslam/mapping/map.hpp"
#include "vslam/tracking/frame.hpp"
#include "vslam/tracking/key_frame.hpp"

#include <vector>

namespace vslam
{

class FeatureMatcher
{
public:
  FeatureMatcher(float nn_dist_ratio);

  void BoWSearch(
    const KeyFrame & key_frame, const Frame & curr_frame,
    std::vector<MapPoint> & matching_points);

  void ProjectionSearch(
    const Frame & curr_frame, std::vector<MapPoint> & matching_points);

private:
  float nn_dist_ratio_;
};

}  // vslam

#endif  // VSLAM__FEATURE_MATCHER_HPP_
