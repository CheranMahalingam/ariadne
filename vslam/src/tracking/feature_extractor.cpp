#include "cuda_vslam_ros/tracking/feature_extractor.hpp"

namespace vslam
{

FeatureExtractor::FeatureExtractor(int orb_levels, float orb_scale_factor, int orb_num_features)
: orb_levels_(orb_levels),
  orb_scale_factor_(orb_scale_factor),
  orb_num_features_(orb_num_features)
{
}

void FeatureExtractor::ComputeFeaturesAndDescriptors(
  cv::InputArray img, cv::InputArray mask,
  std::vector<cv::KeyPoint> & keypoints, cv::OutputArray descriptors)
{
}

void FeatureExtractor::computeKeypoints()
{
}

void FeatureExtractor::computeDescriptors()
{
}

}  // vslam
