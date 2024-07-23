#ifndef VSLAM__VSLAM_HPP_
#define VSLAM__VSLAM_HPP_

#include <foxglove_msgs/msg/pose_in_frame.hpp>
#include <opencv2/core.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <memory>
#include <thread>

namespace vslam
{

namespace sync_policies = message_filters::sync_policies;

using ApproximateTimeSyncPolicy = message_filters::Synchronizer<sync_policies::ApproximateTime<sensor_msgs::msg::Image,
    sensor_msgs::msg::Image>>;

class Canvas;
class LocalMapper;
class Tracker;

class VSLAMNode : public rclcpp::Node
{
public:
  static constexpr int MAX_QUEUE_SIZE = 10;

  VSLAMNode(const rclcpp::NodeOptions & options);
  ~VSLAMNode();

private:
  void rgbdImageCallback(
    const sensor_msgs::msg::Image::ConstSharedPtr rgb_image,
    const sensor_msgs::msg::Image::ConstSharedPtr depth_image);

  void drawLoop();

  std::unique_ptr<LocalMapper> local_mapper_;
  std::unique_ptr<Canvas> canvas_;
  std::unique_ptr<Tracker> tracker_;

  std::jthread lm_thread_;
  std::jthread draw_thread_;

  message_filters::Subscriber<sensor_msgs::msg::Image> rgb_sub_;
  message_filters::Subscriber<sensor_msgs::msg::Image> depth_sub_;
  std::unique_ptr<ApproximateTimeSyncPolicy> camera_syncer_;

  // Publishers used for visual debugging.
  rclcpp::Publisher<foxglove_msgs::msg::PoseInFrame>::SharedPtr debug_pose_pub_;
};

}  // vslam

#endif  // VSLAM__VSLAM_HPP_
