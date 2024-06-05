#include "cuda_vslam_ros/tracking/feature_extractor.hpp"

namespace vslam
{

FeatureExtractor::FeatureExtractor(int orb_scale_pyramid_levels)
: orb_scale_pyramid_levels_(orb_scale_pyramid_levels)
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
