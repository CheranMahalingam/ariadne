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

  // Distance between IR distance sensor and camera in mm.
  float depth_baseline;
};

}  // vslam

#endif  // VSLAM__CAMERA_UTILS_HPP_
