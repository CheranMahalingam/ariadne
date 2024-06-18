#include "vslam/tools/feature_plot.hpp"
#include "vslam/tracking/feature_extractor.hpp"

#include <iostream>
#include <getopt.h>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

FeaturePlot::FeaturePlot(std::string input_dir, std::string output_dir)
: input_dir_(std::filesystem::path(input_dir)),
  output_dir_(std::filesystem::path(output_dir))
{
}

void FeaturePlot::Plot()
{
  for (auto & dir_entry:std::filesystem::directory_iterator(input_dir_)) {
    if (!dir_entry.is_regular_file()) {
      continue;
    }

    auto image = cv::imread(dir_entry.path().string(), cv::IMREAD_UNCHANGED);
    vslam::FeatureExtractor extractor(8, 1.2, 1000, 20, 7);

    std::vector<cv::KeyPoint> key_points;
    cv::Mat descriptors;
    auto grey = image;
    cv::cvtColor(grey, grey, cv::COLOR_RGB2GRAY);
    extractor.ComputeFeatures(grey, key_points, descriptors);
    for (auto kp:key_points) {
      cv::circle(image, kp.pt, 3, cv::Scalar(0, 255, 0), cv::FILLED);
    }

    auto filename = dir_entry.path().filename().string();
    auto output_path = output_dir_ / filename;
    std::cout << output_path << "\n";
    cv::imwrite(output_path.string(), image);
  }
}

void help()
{
  std::cout <<
    "--input-dir <i>:  Set input directory containing RGB images\n"
    "--output-dir <o>: Set output directory for annotated images\n"
    "--help:           Show help\n";
  exit(1);
}

int main(int argc, char ** argv)
{
  std::string input_dir;
  std::string output_dir;
  option long_opts[] = {
    {"input-dir", required_argument, nullptr, 'i'},
    {"output-dir", required_argument, nullptr, 'o'},
    {"help", no_argument, nullptr, 'h'},
    {nullptr, no_argument, nullptr, 0}
  };

  while (true) {
    int opt = getopt_long(argc, argv, "i:o:h", long_opts, nullptr);
    if (opt == -1) {
      break;
    }

    switch (opt) {
      case 'i':
        input_dir = std::string(optarg);
        break;
      case 'o':
        output_dir = std::string(optarg);
        break;
      case 'h':
      case '?':
      default:
        help();
        break;
    }
  }

  FeaturePlot plotter(input_dir, output_dir);
  plotter.Plot();
  return 0;
}
