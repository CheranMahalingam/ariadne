#ifndef VSLAM__CAMERA_UTILS_HPP_
#define VSLAM__CAMERA_UTILS_HPP_

namespace vslam
{

struct CameraParams
{
  // Camera intrinsics parameters
  float fx;
  float fy;
  float cx;
  float cy;

  int width;
  int height;

  int fps;

  float depth_threshold;
  // Distance between IR distance sensor and camera in mm.
  float depth_baseline;
  float depth_map_factor;
};

}  // vslam

#endif  // VSLAM__CAMERA_UTILS_HPP_
