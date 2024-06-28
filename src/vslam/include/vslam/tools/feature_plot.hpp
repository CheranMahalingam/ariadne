#ifndef VSLAM__FEATURE_PLOT_HPP_
#define VSLAM__FEATURE_PLOT_HPP_

#include <filesystem>

class FeaturePlot
{
public:
  FeaturePlot(std::string input_dir, std::string output_dir);

  void PlotFeatures();
  void PlotMatches();

private:
  std::filesystem::path input_dir_;
  std::filesystem::path output_dir_;
};

#endif  // VSLAM__FEATURE_PLOT_HPP_
