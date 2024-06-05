#include "cuda_vslam_ros/vslam.hpp"

#include <cv_bridge/cv_bridge.h>

namespace vslam
{

using std::placeholders::_1;

VSLAMNode::VSLAMNode(const rclcpp::NodeOptions & options)
: Node("vslam", options)
{
  this->declare_parameter("orb_scale_pyramid_levels", 8);
  this->declare_parameter("orb_pyramid_scale_factor", 1.2);
  this->declare_parameter("orb_num_features", 2000);
  this->declare_parameter("orb_initial_FAST_threshold", 20);
  this->declare_parameter("orb_min_FAST_threshold", 7);
  this->declare_parameter("camera_width", 1920);
  this->declare_parameter("camera_height", 1080);

  this->get_parameter("orb_scale_pyramid_levels", orb_scale_pyramid_levels_);
  this->get_parameter("orb_pyramid_scale_factor", orb_pyramid_scale_factor_);
  this->get_parameter("orb_num_features", orb_num_features_);
  this->get_parameter("orb_initial_FAST_threshold", orb_initial_FAST_threshold_);
  this->get_parameter("orb_min_FAST_threshold", orb_min_FAST_threshold_);
  this->get_parameter("camera_width", camera_width_);
  this->get_parameter("camera_height", camera_height_);

  extractor_ = std::make_unique<FeatureExtractor>(
    orb_scale_pyramid_levels_,
    orb_pyramid_scale_factor_,
    orb_num_features_,
    orb_initial_FAST_threshold_,
    orb_min_FAST_threshold_);

  rgb_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
    "rgb_image", 10, std::bind(&VSLAMNode::rgbImageCallback, this, _1));
}

void VSLAMNode::rgbImageCallback(const sensor_msgs::msg::Image::ConstSharedPtr & msg) const
{
  auto cv_ptr = cv_bridge::toCvCopy(msg, msg->encoding);
  std::vector<cv::KeyPoint> key_points;
  cv::Mat descriptors;
  extractor_->ComputeFeatures(cv_ptr->image, key_points, descriptors);
}

}  // vslam
