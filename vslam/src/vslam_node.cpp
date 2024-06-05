#include "cuda_vslam_ros/vslam.hpp"

#include <rclcpp/rclcpp.hpp>

#include <memory>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  rclcpp::spin(std::make_shared<vslam::VSLAMNode>(options));
  rclcpp::shutdown();
  return 0;
}
