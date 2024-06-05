#include "cuda_vslam_ros/vslam.hpp"

namespace vslam
{

using std::placeholders::_1;

VSLAMNode::VSLAMNode(const rclcpp::NodeOptions & options)
: Node("vslam", options)
{
  this->declare_parameter("orb_scale_pyramid_levels", 8);
  this->declare_parameter("orb_pyramid_scale_factor", 1.2);
  this->declare_parameter("orb_num_features", 2000);

  this->get_parameter("orb_scale_pyramid_levels", orb_scale_pyramid_levels_);
  this->get_parameter("orb_pyramid_scale_factor", orb_pyramid_scale_factor_);
  this->get_parameter("orb_num_features", orb_num_features_);

  rgb_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
    "rgb_image", 10, std::bind(&VSLAMNode::rgb_callback, this, _1));
}

void VSLAMNode::rgb_callback(const sensor_msgs::msg::Image & msg) const
{}

}  // vslam
