#ifndef CUDA_VSLAM_ROS__VSLAM_NODE_HPP_
#define CUDA_VSLAM_ROS__VSLAM_NODE_HPP_

#include <rclcpp/rclcpp.hpp>

namespace vslam
{

class VSLAMNode : public rclcpp::Node
{
public:
  VSLAMNode(const rclcpp::NodeOptions & options);
};

}  // vslam

#endif  // CUDA_VSLAM_ROS__VSLAM_NODE_HPP_
