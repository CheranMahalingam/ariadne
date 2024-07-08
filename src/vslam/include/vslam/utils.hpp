#ifndef VSLAM__UTILS_HPP_
#define VSLAM__UTILS_HPP_

#include <opencv2/core.hpp>

namespace vslam
{

struct CameraParams
{
  // Camera intrinsics parameters
  float fx;
  float fy;
  float cx;
  float cy;

  // Camera intrinsics matrix
  cv::Mat k;

  int width;
  int height;

  int fps;

  float depth_threshold;
  // Distance between IR distance sensor and camera in mm.
  float depth_baseline;
  float depth_map_factor;

  void CreateIntrinsics();
};

struct Pose
{
  // Transformations from world to camera coordinate frame.
  cv::Mat transform_cw;
  cv::Mat rotation_cw;
  cv::Mat translation_cw;

  // Transformations from camera to world coordinate frame.
  cv::Mat transform_wc;
  cv::Mat rotation_wc;
  cv::Mat translation_wc;

  bool initialized;

  Pose();
  void SetPose(cv::Mat pose_cw);
};

int hamming_distance(const cv::Mat & a, const cv::Mat & b);

}  // vslam

#endif  // VSLAM__UTILS_HPP_
