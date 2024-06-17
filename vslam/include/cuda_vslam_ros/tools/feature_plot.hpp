#ifndef CUDA_VSLAM_ROS__FEATURE_PLOT_HPP_
#define CUDA_VSLAM_ROS__FEATURE_PLOT_HPP_

#include <string>

class FeaturePlot
{
public:
  FeaturePlot(std::string image_path);

  void Plot(std::string output_path);

private:
  std::string input_path_;
};

#endif  // CUDA_VSLAM_ROS__FEATURE_PLOT_HPP_
