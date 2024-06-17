#include "cuda_vslam_ros/vslam.hpp"

#include <cv_bridge/cv_bridge.h>
#include <foxglove_msgs/msg/circle_annotation.hpp>

namespace vslam
{

int VSLAMNode::debug_frame_count = 0;

VSLAMNode::VSLAMNode()
: Node("vslam")
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

  orb_features_debug_pub_ = this->create_publisher<foxglove_msgs::msg::ImageAnnotations>(
    "/debug/localization/orb_features", MAX_QUEUE_SIZE);

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

  debugORBFeatures(key_points, timestamp);
}

void VSLAMNode::debugORBFeatures(
  const std::vector<cv::KeyPoint> & key_points, rclcpp::Time timestamp)
{
  if (debug_frame_count % 5 == 0) {
    auto annotation_msg = foxglove_msgs::msg::ImageAnnotations();
    RCLCPP_INFO(this->get_logger(), "LEN %ld", key_points.size());
    for (int i = 0; i < std::min(int(key_points.size()), 100); i++) {
      auto kp = key_points[i];
      auto circle_msg = foxglove_msgs::msg::CircleAnnotation();
      circle_msg.timestamp = timestamp;
      circle_msg.position.x = kp.pt.x;
      circle_msg.position.y = kp.pt.y;
      // RCLCPP_INFO(this->get_logger(), "X %f Y %f", kp.pt.x, kp.pt.y);
      // circle_msg.thickness = 3;
      circle_msg.fill_color.r = 0;
      circle_msg.fill_color.g = 1;
      circle_msg.fill_color.b = 0;
      circle_msg.fill_color.a = 1;
      // circle_msg.outline_color.r = 1;
      // circle_msg.outline_color.g = 1;
      // circle_msg.outline_color.b = 1;
      // circle_msg.outline_color.a = 1;
      circle_msg.diameter = 5;
      annotation_msg.circles.push_back(circle_msg);
    }
    orb_features_debug_pub_->publish(annotation_msg);
  }
  debug_frame_count++;
}

}  // vslam
