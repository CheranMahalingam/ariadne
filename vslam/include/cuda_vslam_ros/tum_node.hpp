#ifndef CUDA_VSLAM_ROS__TUM_NODE_HPP_
#define CUDA_VSLAM_ROS__TUM_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <memory>

class TUMNode : public rclcpp::Node
{
public:
  static constexpr int MAX_QUEUE_SIZE = 10;

  TUMNode(const rclcpp::NodeOptions & options);

private:
  void publishImage();

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr rgb_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_pub_;
};

#endif  // CUDA_VSLAM_ROS__TUM_NODE_HPP_
