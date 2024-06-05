#ifndef CUDA_VSLAM_ROS__VSLAM_NODE_HPP_
#define CUDA_VSLAM_ROS__VSLAM_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

namespace vslam
{

class VSLAMNode : public rclcpp::Node
{
public:
  VSLAMNode(const rclcpp::NodeOptions & options);

private:
  void rgb_callback(const sensor_msgs::msg::Image & msg) const;

  // Parameters
  int orb_scale_pyramid_levels_;

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr rgb_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
};

}  // vslam

#endif  // CUDA_VSLAM_ROS__VSLAM_NODE_HPP_
