#include "vslam/utils.hpp"
#include "vslam/visualization/canvas.hpp"

namespace vslam
{

Canvas::Canvas()
: pose_modified_(false)
{
}

void Canvas::SetCameraPose(const cv::Mat & pose_cw)
{
  std::lock_guard<std::mutex> lock(canvas_mutex_);
  pose_ = pose_cw.clone();
  pose_modified_ = true;
}

std::optional<foxglove_msgs::msg::PoseInFrame> Canvas::GetCameraPose(
  rclcpp::Time t)
{
  cv::Mat pose;
  {
    std::lock_guard<std::mutex> lock(canvas_mutex_);
    if (!pose_modified_) {
      return {};
    }
    pose_modified_ = false;
    pose = pose_.clone();
  }

  auto g2o_pose = convertToSE3Quat(pose);
  auto translation = g2o_pose.translation();
  auto rotation = g2o_pose.rotation();

  foxglove_msgs::msg::PoseInFrame pose_marker;
  pose_marker.timestamp = t;
  pose_marker.frame_id = "world";
  pose_marker.pose.position.x = translation(0);
  pose_marker.pose.position.y = translation(1);
  pose_marker.pose.position.z = translation(2);
  pose_marker.pose.orientation.x = rotation.x();
  pose_marker.pose.orientation.y = rotation.y();
  pose_marker.pose.orientation.z = rotation.z();
  pose_marker.pose.orientation.w = rotation.w();
  return pose_marker;
}

}  // vslam
