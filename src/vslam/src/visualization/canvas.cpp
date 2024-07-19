#include "vslam/utils.hpp"
#include "vslam/visualization/canvas.hpp"

#include <algorithm>
#include <chrono>
#include <thread>

namespace vslam
{

Canvas::Canvas(
  rclcpp::Publisher<foxglove_msgs::msg::PoseInFrame>::SharedPtr pose_pub)
: pose_pub_(pose_pub)
{
}

void Canvas::Run()
{
  while (true) {
    auto start = std::chrono::high_resolution_clock::now();

    publishPose();

    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> elapsed = end - start;
    auto sleep_duration = std::max(
      std::chrono::duration<double, std::milli>(0),
      std::chrono::milliseconds(100) - elapsed);
    std::this_thread::sleep_for(sleep_duration);
  }
}

void Canvas::SetCameraPose(const cv::Mat & pose_cw)
{
  pose_ = pose_cw.clone();
}

void Canvas::publishPose()
{
  auto g2o_pose = convertToSE3Quat(pose_);
  auto translation = g2o_pose.translation();
  auto rotation = g2o_pose.rotation();
  foxglove_msgs::msg::PoseInFrame pose_marker;
  // pose_marker.timestamp = 
  pose_marker.frame_id = "world";
  pose_marker.pose.position.x = translation;
  pose_marker.pose.position.y = translation;
  pose_marker.pose.position.z = translation;
  pose_pub_->publish(pose_marker);
}

}  // vslam
