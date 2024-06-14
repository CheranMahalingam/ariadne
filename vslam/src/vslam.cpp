#include "cuda_vslam_ros/vslam.hpp"

#include <cv_bridge/cv_bridge.h>

namespace vslam
{

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

  annotation_debug_pub_ = this->create_publisher<foxglove_msgs::msg::RawImage>(
    "/debug/slam/features", MAX_QUEUE_SIZE);

  rgb_sub_.subscribe(this, "/sensing/camera/rgb");
  depth_sub_.subscribe(this, "/sensing/camera/depth");
  camera_syncer_ = std::make_unique<ApproximateTimeSyncPolicy>(MAX_QUEUE_SIZE);
  camera_syncer_->connectInput(rgb_sub_, depth_sub_);
  camera_syncer_->registerCallback(
    std::bind(
      &VSLAMNode::rgbdImageCallback, this,
      std::placeholders::_1, std::placeholders::_2));
}

void VSLAMNode::rgbdImageCallback(
  const sensor_msgs::msg::Image::ConstSharedPtr rgb_image,
  const sensor_msgs::msg::Image::ConstSharedPtr depth_image)
{
  auto cv_ptr_rgb = cv_bridge::toCvShare(rgb_image);
  auto cv_ptr_depth = cv_bridge::toCvShare(depth_image);
  track(cv_ptr_rgb->image, cv_ptr_depth->image, cv_ptr_rgb->header.stamp);
}

void VSLAMNode::track(
  const cv::Mat & rgb, const cv::Mat & depth, rclcpp::Time timestamp)
{
  auto grey = rgb;
  cv::cvtColor(grey, grey, cv::COLOR_RGB2GRAY);

  std::vector<cv::KeyPoint> key_points;
  cv::Mat descriptors;
  extractor_->ComputeFeatures(grey, key_points, descriptors);

  auto msg = foxglove_msgs::msg::RawImage();
  annotation_debug_pub_->publish(msg);
}

}  // vslam
