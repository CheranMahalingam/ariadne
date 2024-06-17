#include "cuda_vslam_ros/tools/feature_plot.hpp"
#include "cuda_vslam_ros/tracking/feature_extractor.hpp"

// #include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <filesystem>

FeaturePlot::FeaturePlot(std::string image_path)
: input_path_(image_path)
{
}

void FeaturePlot::Plot(std::string output_dir)
{
  auto image = cv::imread(input_path_, cv::IMREAD_UNCHANGED);
  vslam::FeatureExtractor extractor(1, 1.2, 500, 20, 7);

  std::vector<cv::KeyPoint> key_points;
  cv::Mat descriptors;
  auto grey = image;
  cv::cvtColor(grey, grey, cv::COLOR_RGB2GRAY);
  extractor.ComputeFeatures(grey, key_points, descriptors);
  for (auto kp:key_points) {
    cv::circle(image, kp.pt, 3, cv::Scalar(0, 255, 0));
  }

  std::filesystem::path input(input_path_);
  auto output_path = std::filesystem::path(output_dir).append(input.filename().string());
  cv::imwrite(output_path.string(), image);
}

int main(int argc, char ** argv)
{
  std::string image_path = "data/images/test.jpg";
  FeaturePlot plotter(image_path);
  plotter.Plot("data/feature_plots");
  return 0;
}
