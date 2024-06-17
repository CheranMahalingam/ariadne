#ifndef VSLAM__VSLAM_HPP_
#define VSLAM__VSLAM_HPP_

#include <rclcpp/rclcpp.hpp>

namespace vslam
{

class VSLAMNode : public rclcpp::Node
{
public:
  VSLAMNode(const rclcpp::NodeOptions & options);
};

}  // vslam

#endif  // VSLAM__VSLAM_HPP_
