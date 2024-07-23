#include "vslam/mapping/map.hpp"
#include "vslam/mapping/map_point.hpp"
#include "vslam/optimization/optimizer.hpp"
#include "vslam/tracking/key_frame.hpp"
#include "vslam/utils.hpp"

#include "g2o/core/block_solver.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/solvers/dense/linear_solver_dense.h"
#include "g2o/solvers/eigen/linear_solver_eigen.h"
#include <rclcpp/rclcpp.hpp>

#include <vector>

namespace vslam
{

using BlockSolver = g2o::BlockSolver_6_3;

Optimizer::Optimizer() {}

void Optimizer::LocalBundleAdjustment(std::shared_ptr<KeyFrame> kf)
{
  std::vector<std::shared_ptr<KeyFrame>> local_key_frames = {kf};
  auto neighbour_kfs = kf->GetCovisibleKeyFrames();
  for (auto neighbour_kf:neighbour_kfs) {
    if (!neighbour_kf->Culled()) {
      local_key_frames.push_back(neighbour_kf);
    }
  }

  std::vector<std::shared_ptr<MapPoint>> local_map_points;
  for (auto local_kf:local_key_frames) {
    auto map_points = local_kf->GetMapPoints();
    for (auto mp:map_points) {
      if (mp != nullptr && !mp->Culled() &&
        std::find(
          local_map_points.begin(), local_map_points.end(), mp
        ) == local_map_points.end())
      {
        local_map_points.push_back(mp);
      }
    }
  }

  std::vector<std::shared_ptr<KeyFrame>> fixed_cameras;
  for (auto mp:local_map_points) {
    auto observations = mp->GetObservations();
    for (auto [obs_kf, idx]:observations) {
      if (!obs_kf->Culled() && std::find(
          local_key_frames.begin(), local_key_frames.end(), obs_kf
        ) == local_key_frames.end() &&
        std::find(
          fixed_cameras.begin(), fixed_cameras.end(), obs_kf
        ) == fixed_cameras.end())
      {
        fixed_cameras.push_back(obs_kf);
      }
    }
  }

  g2o::SparseOptimizer optimizer;
  // Raw pointers can be safely used with g2o without leaking memory. g2o is
  // responsible for managing the lifetime of objects.
  auto solver = new g2o::OptimizationAlgorithmLevenberg(
    std::make_unique<BlockSolver>(
      std::make_unique<g2o::LinearSolverEigen<BlockSolver::PoseMatrixType>>()));
  optimizer.setAlgorithm(solver);

  int max_kf_id = 0;
  for (auto local_kf:local_key_frames) {
    addVertex(optimizer, local_kf.get(), max_kf_id);
  }
  for (auto fixed_kf:fixed_cameras) {
    addVertex(optimizer, fixed_kf.get(), max_kf_id);
  }

  std::vector<g2o::EdgeSE3ProjectXYZ *> edges;
  std::vector<std::shared_ptr<KeyFrame>> kf_edges;
  std::vector<std::shared_ptr<MapPoint>> mp_edges;
  for (auto local_mp:local_map_points) {
    auto vertex = new g2o::VertexPointXYZ();
    vertex->setEstimate(
      convertToVector(local_mp->GetWorldPos()));
    // Increment vertex id by max_kf_id to avoid conflicts with key frames.
    int mp_vertex_id = local_mp->curr_id + max_kf_id + 1;
    vertex->setId(mp_vertex_id);
    vertex->setMarginalized(true);
    optimizer.addVertex(vertex);

    auto observations = local_mp->GetObservations();
    for (auto [obs_kf, idx]:observations) {
      if (obs_kf->Culled()) {
        continue;
      }

      auto edge = new g2o::EdgeSE3ProjectXYZ();
      edge->setVertex(
        0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(
          optimizer.vertex(mp_vertex_id)));
      edge->setVertex(
        1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(
          optimizer.vertex(obs_kf->curr_id)));

      auto kp = obs_kf->key_points[idx];
      addEdge<g2o::EdgeSE3ProjectXYZ>(optimizer, obs_kf.get(), kp, edge);

      edges.push_back(edge);
      kf_edges.push_back(obs_kf);
      mp_edges.push_back(local_mp);
    }
  }

  optimizer.initializeOptimization();
  optimizer.optimize(LEVENBERG_OPTIMIZATION_ITERATIONS / 2);

  for (int i = 0; i < int(edges.size()); i++) {
    auto mp = mp_edges[i];
    if (mp->Culled()) {
      continue;
    }

    auto edge = edges[i];
    if (edge->chi2() > G2O_EDGE_CHI2_THRESHOLD || !edge->isDepthPositive()) {
      edge->setLevel(1);
    }
    edge->setRobustKernel(0);
  }

  optimizer.initializeOptimization(0);
  optimizer.optimize(LEVENBERG_OPTIMIZATION_ITERATIONS);

  std::vector<std::pair<std::shared_ptr<KeyFrame>, std::shared_ptr<MapPoint>>> outliers;
  for (int i = 0; i < int(edges.size()); i++) {
    auto mp = mp_edges[i];
    if (mp->Culled()) {
      continue;
    }

    auto edge = edges[i];
    if (edge->chi2() > G2O_EDGE_CHI2_THRESHOLD || !edge->isDepthPositive()) {
      outliers.push_back(std::make_pair(kf_edges[i], mp_edges[i]));
    }
  }

  for (auto [outlier_kf, outlier_mp]:outliers) {
    auto obs_idx = outlier_mp->GetObservationIndex(outlier_kf);
    assert(obs_idx != -1);
    outlier_kf->EraseMapPoint(obs_idx);
    outlier_mp->EraseObservation(outlier_kf);
  }

  for (auto local_kf:local_key_frames) {
    auto optimized_vertex = static_cast<g2o::VertexSE3Expmap *>(
      optimizer.vertex(local_kf->curr_id));
    auto pose = convertToMat(optimized_vertex->estimate());
    local_kf->SetPose(pose);
  }

  for (auto local_mp:local_map_points) {
    auto optimized_vertex = static_cast<g2o::VertexPointXYZ *>(
      optimizer.vertex(local_mp->curr_id + max_kf_id + 1));
    auto pos = convertToMat(optimized_vertex->estimate());
    local_mp->SetWorldPos(pos);
    local_mp->UpdateObservations();
  }
}

int Optimizer::PoseOptimization(std::shared_ptr<Frame> frame)
{
  g2o::SparseOptimizer optimizer;
  auto solver = new g2o::OptimizationAlgorithmLevenberg(
    std::make_unique<BlockSolver>(
      std::make_unique<g2o::LinearSolverDense<BlockSolver::PoseMatrixType>>()));
  optimizer.setAlgorithm(solver);

  auto frame_vertex = new g2o::VertexSE3Expmap();
  frame_vertex->setEstimate(
    convertToSE3Quat(frame->pose.transform_cw));
  frame_vertex->setId(0);
  frame_vertex->setFixed(false);
  optimizer.addVertex(frame_vertex);

  std::vector<g2o::EdgeSE3ProjectXYZOnlyPose *> edges;
  std::vector<int> valid_edges;
  int initial_inliers = 0;
  for (int i = 0; i < int(frame->map_points.size()); i++) {
    auto mp = frame->map_points[i];
    if (mp == nullptr) {
      continue;
    }

    initial_inliers++;
    frame->outliers[i] = false;

    auto edge = new g2o::EdgeSE3ProjectXYZOnlyPose();
    edge->setVertex(
      0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(0)));

    auto world_pos = mp->GetWorldPos();
    edge->Xw[0] = world_pos.at<float>(0);
    edge->Xw[1] = world_pos.at<float>(1);
    edge->Xw[2] = world_pos.at<float>(2);

    auto kp = frame->key_points[i];
    addEdge<g2o::EdgeSE3ProjectXYZOnlyPose>(optimizer, frame.get(), kp, edge);

    edges.push_back(edge);
    valid_edges.push_back(i);
  }

  if (initial_inliers < MINIMUM_VALID_MAP_POINTS) {
    return 0;
  }

  int outliers = 0;
  for (int iteration = 0; iteration < POSE_OPTIMIZATION_ITERATIONS; iteration++) {
    frame_vertex->setEstimate(
      convertToSE3Quat(frame->pose.transform_cw));
    optimizer.initializeOptimization(0);
    optimizer.optimize(LEVENBERG_OPTIMIZATION_ITERATIONS);

    outliers = 0;
    double min_chi2 = -1;
    for (int i = 0; i < int(edges.size()); i++) {
      auto edge = edges[i];
      auto valid_idx = valid_edges[i];
      if (frame->outliers[valid_idx]) {
        edge->computeError();
      }

      auto chi2 = edge->chi2();
      if (chi2 < min_chi2 || min_chi2 == -1) {
        min_chi2 = chi2;
      }
      if (chi2 > G2O_EDGE_CHI2_THRESHOLD) {
        frame->outliers[valid_idx] = true;
        edge->setLevel(1);
        outliers++;
      } else {
        frame->outliers[valid_idx] = false;
        edge->setLevel(0);
      }

      // If previous optimization attempts failed, try again without a Huber
      // kernel.
      if (iteration == POSE_OPTIMIZATION_ITERATIONS / 2) {
        edge->setRobustKernel(0);
      }
    }

    auto optimizer_edges = optimizer.edges().size();
    if (optimizer_edges < 10) {
      break;
    }
  }

  auto optimized_vertex = static_cast<g2o::VertexSE3Expmap *>(
    optimizer.vertex(0));
  auto pose = convertToMat(optimized_vertex->estimate());
  frame->SetPose(pose);
  RCLCPP_INFO(
    rclcpp::get_logger("optimizer"),
    "Found %d inliers and %d outliers", initial_inliers, outliers);
  return initial_inliers - outliers;
}

void Optimizer::addVertex(
  g2o::SparseOptimizer & optimizer, KeyFrame * kf, int & max_kf_id)
{
  auto vertex = new g2o::VertexSE3Expmap();
  vertex->setEstimate(
    convertToSE3Quat(kf->GetPose()));
  vertex->setId(kf->curr_id);
  // Avoid modifying first key frame.
  vertex->setFixed(kf->curr_id == 0);

  optimizer.addVertex(vertex);
  if (kf->curr_id > max_kf_id) {
    max_kf_id = kf->curr_id;
  }
}

}  // vslam
