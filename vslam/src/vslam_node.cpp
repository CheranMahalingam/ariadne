#include "cuda_vslam_ros/vslam.hpp"

#include <rclcpp/rclcpp.hpp>

#include <memory>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<vslam::VSLAMNode>());
  rclcpp::shutdown();
  return 0;
}
