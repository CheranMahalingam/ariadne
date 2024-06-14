#ifndef CUDA_VSLAM_ROS__VSLAM_NODE_HPP_
#define CUDA_VSLAM_ROS__VSLAM_NODE_HPP_

#include "tracking/feature_extractor.hpp"

#include <opencv2/core.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <rclcpp/rclcpp.hpp>
#include <foxglove_msgs/msg/raw_image.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <memory>

namespace vslam
{
namespace sync_policies = message_filters::sync_policies;

using ApproximateTimeSyncPolicy = message_filters::Synchronizer<sync_policies::ApproximateTime<sensor_msgs::msg::Image,
    sensor_msgs::msg::Image>>;

class VSLAMNode : public rclcpp::Node
{
public:
  static constexpr int MAX_QUEUE_SIZE = 10;

  VSLAMNode(const rclcpp::NodeOptions & options);

private:
  void rgbdImageCallback(
    const sensor_msgs::msg::Image::ConstSharedPtr rgb_image,
    const sensor_msgs::msg::Image::ConstSharedPtr depth_image);

  void track(
    const cv::Mat & rgb, const cv::Mat & depth, rclcpp::Time timestamp);

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

  rclcpp::Publisher<foxglove_msgs::msg::RawImage>::SharedPtr annotation_debug_pub_;
  message_filters::Subscriber<sensor_msgs::msg::Image> rgb_sub_;
  message_filters::Subscriber<sensor_msgs::msg::Image> depth_sub_;
  std::unique_ptr<ApproximateTimeSyncPolicy> camera_syncer_;
};

}  // vslam

#endif  // CUDA_VSLAM_ROS__VSLAM_NODE_HPP_
