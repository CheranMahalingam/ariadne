#ifndef CUDA_VSLAM_ROS__VSLAM_NODE_HPP_
#define CUDA_VSLAM_ROS__VSLAM_NODE_HPP_

#include "tracking/feature_extractor.hpp"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

namespace vslam
{

using SyncPolicy = message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image>;

class VSLAMNode : public rclcpp::Node
{
public:
  VSLAMNode(const rclcpp::NodeOptions & options);

private:
  void rgbdImageCallback(
    const sensor_msgs::msg::Image::ConstSharedPtr & rgb_image,
    const sensor_msgs::msg::Image::ConstSharedPtr & depth_image) const;

  // ROS configurable parameters
  int orb_scale_pyramid_levels_;
  float orb_pyramid_scale_factor_;
  int orb_num_features_;
  int orb_initial_FAST_threshold_;
  int orb_min_FAST_threshold_;
  int camera_width_;
  int camera_height_;
  float depth_threshold_;

  std::unique_ptr<FeatureExtractor> extractor_;

  message_filters::Subscriber<sensor_msgs::msg::Image> rgb_sub_;
  message_filters::Subscriber<sensor_msgs::msg::Image> depth_sub_;
  message_filters::Synchronizer<SyncPolicy> camera_sync_;
};

}  // vslam

#endif  // CUDA_VSLAM_ROS__VSLAM_NODE_HPP_
