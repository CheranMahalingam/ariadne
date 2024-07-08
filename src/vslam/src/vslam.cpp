#include "vslam/mapping/local_mapper.hpp"
#include "vslam/mapping/map.hpp"
#include "vslam/tracking/feature_extractor.hpp"
#include "vslam/tracking/tracker.hpp"
#include "vslam/utils.hpp"
#include "vslam/vslam.hpp"

#include "DBoW3/DBoW3.h"
#include <cv_bridge/cv_bridge.h>

namespace vslam
{

VSLAMNode::VSLAMNode(const rclcpp::NodeOptions & options)
: Node("vslam", options)
{
  this->declare_parameter("orb_scale_pyramid_levels", 8);
  this->declare_parameter("orb_pyramid_scale_factor", 1.2);
  this->declare_parameter("orb_num_features", 2000);
  this->declare_parameter("orb_initial_FAST_threshold", 20);
  this->declare_parameter("orb_min_FAST_threshold", 7);
  this->declare_parameter("camera_fx", 500.0);
  this->declare_parameter("camera_fy", 500.0);
  this->declare_parameter("camera_width", 1920);
  this->declare_parameter("camera_height", 1080);
  this->declare_parameter("camera_fps", 30);
  this->declare_parameter("camera_depth_threshold", 40.0);
  this->declare_parameter("camera_depth_baseline_mm", 8.0);
  this->declare_parameter("camera_depth_map_factor", 5000.0);

  FeatureExtractor::ORBParams fp;
  this->get_parameter("orb_scale_pyramid_levels", fp.orb_levels);
  this->get_parameter("orb_pyramid_scale_factor", fp.orb_scale_factor);
  this->get_parameter("orb_num_features", fp.orb_num_features);
  this->get_parameter("orb_initial_FAST_threshold", fp.orb_initial_FAST_threshold);
  this->get_parameter("orb_min_FAST_threshold", fp.orb_min_FAST_threshold);

  CameraParams cp;
  this->get_parameter("camera_fx", cp.fx);
  this->get_parameter("camera_fy", cp.fy);
  this->get_parameter("camera_cx", cp.cx);
  this->get_parameter("camera_cy", cp.cy);
  this->get_parameter("camera_width", cp.width);
  this->get_parameter("camera_height", cp.height);
  this->get_parameter("camera_fps", cp.fps);
  this->get_parameter("camera_depth_threshold", cp.depth_threshold);
  this->get_parameter("camera_depth_baseline_mm", cp.depth_baseline);
  this->get_parameter("camera_depth_map_factor", cp.depth_map_factor);
  cp.CreateIntrinsics();

  std::string vocabulary_path = "data/DBoW3/orbvoc.dbow3";
  DBoW3::Vocabulary v(vocabulary_path);
  if (v.empty()) {
    RCLCPP_ERROR(
      this->get_logger(),
      "Unable to parse BoW vocabulary at %s", vocabulary_path.c_str());
    return;
  } else {
    RCLCPP_INFO(
      this->get_logger(),
      "Successfully loaded BoW vocabulary from %s", vocabulary_path.c_str());
  }

  auto map = std::make_shared<Map>();

  local_mapper_ = std::make_unique<LocalMapper>(map);
  lm_thread_ = std::jthread(&LocalMapper::Run, local_mapper_.get());

  tracker_ = std::make_unique<Tracker>(local_mapper_.get(), map, v, fp, cp);

  rgb_sub_.subscribe(this, "/sensing/camera/rgb");
  depth_sub_.subscribe(this, "/sensing/camera/depth");
  camera_syncer_ = std::make_unique<ApproximateTimeSyncPolicy>(MAX_QUEUE_SIZE);
  camera_syncer_->connectInput(rgb_sub_, depth_sub_);
  camera_syncer_->registerCallback(
    std::bind(
      &VSLAMNode::rgbdImageCallback, this,
      std::placeholders::_1, std::placeholders::_2));
}

VSLAMNode::~VSLAMNode() = default;

void VSLAMNode::rgbdImageCallback(
  const sensor_msgs::msg::Image::ConstSharedPtr rgb_image,
  const sensor_msgs::msg::Image::ConstSharedPtr depth_image)
{
  auto cv_ptr_rgb = cv_bridge::toCvShare(rgb_image);
  auto cv_ptr_depth = cv_bridge::toCvShare(depth_image);
  tracker_->Track(cv_ptr_rgb->image, cv_ptr_depth->image, cv_ptr_rgb->header.stamp);
}

}  // vslam
