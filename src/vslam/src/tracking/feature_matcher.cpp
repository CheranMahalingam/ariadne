#include "vslam/tracking/feature_matcher.hpp"
#include "DBoW3/FeatureVector.h"

namespace vslam
{

FeatureMatcher::FeatureMatcher(float nn_dist_ratio)
: nn_dist_ratio_(nn_dist_ratio)
{
}

void FeatureMatcher::BoWSearch(
  const KeyFrame & key_frame, const Frame & curr_frame,
  std::vector<MapPoint> & matching_points)
{
}

void ProjectionSearch(
  const Frame & curr_frame, std::vector<MapPoint> & matching_points)
{
}

}  // vslam
