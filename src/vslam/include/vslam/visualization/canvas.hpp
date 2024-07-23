#ifndef VSLAM__CANVAS_HPP_
#define VSLAM__CANVAS_HPP_

#include <foxglove_msgs/msg/pose_in_frame.hpp>
#include <opencv2/core.hpp>
#include <rclcpp/rclcpp.hpp>

#include <mutex>
#include <optional>

namespace vslam
{

class Canvas
{
public:
  Canvas();

  void SetCameraPose(const cv::Mat & pose_cw);
  std::optional<foxglove_msgs::msg::PoseInFrame> GetCameraPose(
    rclcpp::Time t);

private:
  std::mutex canvas_mutex_;

  cv::Mat pose_;
  bool pose_modified_;
};

}  // vslam

#endif  // VSLAM__CANVAS_HPP_
