#include "cuda_vslam_ros/tum_node.hpp"

TUMNode::TUMNode(const rclcpp::NodeOptions & options)
: Node("tum", options)
{
  rgb_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
    "/sensing/camera/rgb", MAX_QUEUE_SIZE);
  depth_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
    "/sensing/camera/depth", MAX_QUEUE_SIZE);
}

void TUMNode::publishImage()
{
  auto rgb_image = sensor_msgs::msg::Image();
  auto depth_image = sensor_msgs::msg::Image();

  rgb_pub_->publish(rgb_image);
  depth_pub_->publish(depth_image);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  rclcpp::spin(std::make_shared<TUMNode>(options));
  rclcpp::shutdown();
  return 0;
}
