#ifndef VSLAM__OPTIMIZER_HPP_
#define VSLAM__OPTIMIZER_HPP_

#include "vslam/tracking/frame.hpp"

#include "g2o/core/robust_kernel_impl.h"
#include "g2o/core/sparse_optimizer.h"

#include <memory>

namespace vslam
{

class KeyFrame;
class Map;

class Optimizer
{
public:
  static constexpr float G2O_EDGE_CHI2_THRESHOLD = 5.991;
  static constexpr int POSE_OPTIMIZATION_ITERATIONS = 4;
  static constexpr int LEVENBERG_OPTIMIZATION_ITERATIONS = 10;
  static constexpr int MINIMUM_VALID_MAP_POINTS = 3;

  Optimizer();

  /**
   * Applies bundle adjustment to the specified key frame by building a graph
   * containing frames from the covisibility graph, observed map points, and
   * neighbouring key frames. Corrects the pose of all local key frames and
   * positions of map points. Removes links between key frames and map points
   * that are identified as outliers.
   */
  void LocalBundleAdjustment(std::shared_ptr<KeyFrame> kf);

  /**
   * Applies pose graph optimization on the specified frame and uses it's
   * observed map points as constraints to correct the frame's pose.
   */
  int PoseOptimization(std::shared_ptr<Frame> frame);

private:
  void addVertex(
    g2o::SparseOptimizer & optimizer, const KeyFrame * kf, int & max_kf_id);

  template<typename T>
  void addEdge(
    g2o::SparseOptimizer & optimizer, const FrameBase * frame,
    const cv::KeyPoint & kp, T * edge);
};

template<typename T>
void Optimizer::addEdge(
  g2o::SparseOptimizer & optimizer, const FrameBase * frame,
  const cv::KeyPoint & kp, T * edge)
{
  Eigen::Matrix<double, 2, 1> pixel_pos(kp.pt.x, kp.pt.y);
  edge->setMeasurement(pixel_pos);

  auto inv_scale = 1.0 / frame->image_scale_factors[kp.octave];
  edge->setInformation(Eigen::Matrix2d::Identity() * inv_scale * inv_scale);

  auto huber_kernel = new g2o::RobustKernelHuber();
  huber_kernel->setDelta(std::sqrt(G2O_EDGE_CHI2_THRESHOLD));
  edge->setRobustKernel(huber_kernel);

  edge->fx = frame->camera_params.fx;
  edge->fy = frame->camera_params.fy;
  edge->cx = frame->camera_params.cx;
  edge->cy = frame->camera_params.cy;

  optimizer.addEdge(edge);
}


}  // vslam

#endif  // VSLAM__OPTIMIZER_HPP_
