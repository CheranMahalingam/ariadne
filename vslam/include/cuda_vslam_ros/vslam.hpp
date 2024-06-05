#ifndef CUDA_VSLAM_ROS__VSLAM_NODE_HPP_
#define CUDA_VSLAM_ROS__VSLAM_NODE_HPP_

#include "tracking/feature_extractor.hpp"

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

#include <memory>

namespace vslam
{

class VSLAMNode : public rclcpp::Node
{
public:
  VSLAMNode(const rclcpp::NodeOptions & options);

private:
  void rgbImageCallback(const sensor_msgs::msg::Image::ConstSharedPtr & msg) const;

  // ROS configurable parameters
  int orb_scale_pyramid_levels_;
  float orb_pyramid_scale_factor_;
  int orb_num_features_;
  int orb_initial_FAST_threshold_;
  int orb_min_FAST_threshold_;
  int camera_width_;
  int camera_height_;

  std::unique_ptr<FeatureExtractor> extractor_;

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr rgb_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
};

}  // vslam

#endif  // CUDA_VSLAM_ROS__VSLAM_NODE_HPP_
