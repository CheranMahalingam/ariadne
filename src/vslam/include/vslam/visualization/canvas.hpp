#ifndef VSLAM__CANVAS_HPP_
#define VSLAM__CANVAS_HPP_

#include <foxglove_msgs/msg/pose_in_frame.hpp>
#include <opencv2/core.hpp>
#include <rclcpp/rclcpp.hpp>

namespace vslam
{

class Canvas
{
public:
  Canvas(
    rclcpp::Publisher<foxglove_msgs::msg::PoseInFrame>::SharedPtr pose_pub);

  void Run();

  void SetCameraPose(const cv::Mat & pose_cw);

private:
  void publishPose();

  cv::Mat pose_;

  rclcpp::Publisher<foxglove_msgs::msg::PoseInFrame>::SharedPtr pose_pub_;
};

}  // vslam

#endif  // VSLAM__CANVAS_HPP_
