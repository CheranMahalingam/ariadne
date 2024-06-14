#include "cuda_vslam_ros/vslam.hpp"

#include <cv_bridge/cv_bridge.h>

namespace vslam
{

using std::placeholders::_1;
using std::placeholders::_2;
using SyncPolicy = message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image>;

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
  this->declare_parameter("depth_threshold", 40.0);

  this->get_parameter("orb_scale_pyramid_levels", orb_scale_pyramid_levels_);
  this->get_parameter("orb_pyramid_scale_factor", orb_pyramid_scale_factor_);
  this->get_parameter("orb_num_features", orb_num_features_);
  this->get_parameter("orb_initial_FAST_threshold", orb_initial_FAST_threshold_);
  this->get_parameter("orb_min_FAST_threshold", orb_min_FAST_threshold_);
  this->get_parameter("camera_width", camera_width_);
  this->get_parameter("camera_height", camera_height_);
  this->get_parameter("depth_threshold", depth_threshold_);

  extractor_ = std::make_unique<FeatureExtractor>(
    orb_scale_pyramid_levels_,
    orb_pyramid_scale_factor_,
    orb_num_features_,
    orb_initial_FAST_threshold_,
    orb_min_FAST_threshold_);

  rgb_sub_.subscribe(this, "/sensor/rgb_camera");
  depth_sub_.subscribe(this, "/sensors/depth_camera");
  camera_sync_(SyncPolicy(10), rgb_sub_, depth_sub_);
  camera_sync_.registerCallback(std::bind(&VSLAMNode::rgbdImageCallback, this, _1, _2));
}

void VSLAMNode::rgbdImageCallback(
  const sensor_msgs::msg::Image::ConstSharedPtr & rgb_image,
  const sensor_msgs::msg::Image::ConstSharedPtr & depth_image) const
{
  auto cv_ptr = cv_bridge::toCvCopy(rgb_image, rgb_image->encoding);
  std::vector<cv::KeyPoint> key_points;
  cv::Mat descriptors;
  extractor_->ComputeFeatures(cv_ptr->image, key_points, descriptors);
}

}  // vslam
