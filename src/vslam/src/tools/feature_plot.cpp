#include "vslam/tools/feature_plot.hpp"
#include "vslam/tracking/feature_extractor.hpp"

#include <getopt.h>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <iostream>
#include <set>

FeaturePlot::FeaturePlot(std::string input_dir, std::string output_dir)
: input_dir_(std::filesystem::path(input_dir)),
  output_dir_(std::filesystem::path(output_dir))
{
}

void FeaturePlot::PlotFeatures()
{
  for (auto & dir_entry:std::filesystem::directory_iterator(input_dir_)) {
    if (!dir_entry.is_regular_file()) {
      continue;
    }

    auto image = cv::imread(dir_entry.path().string(), cv::IMREAD_UNCHANGED);
    vslam::FeatureExtractor::ORBParams params = {
      .orb_levels = 8,
      .orb_scale_factor = 1.2,
      .orb_num_features = 1000,
      .orb_initial_FAST_threshold = 20,
      .orb_min_FAST_threshold = 7,
    };
    vslam::FeatureExtractor extractor(params);

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
    cv::imwrite(output_path.string(), image);
  }
}

void FeaturePlot::PlotMatches()
{
  std::set<std::filesystem::path> sorted_filepaths;
  for (auto & dir_entry:std::filesystem::directory_iterator(input_dir_)) {
    if (!dir_entry.is_regular_file()) {
      continue;
    }
    sorted_filepaths.insert(dir_entry.path());
  }

  auto path_it = sorted_filepaths.begin();
  auto prev_path = path_it;
  path_it++;
  for (; path_it != sorted_filepaths.end(); path_it++) {
    std::cout << "PREV " << prev_path->string() << " CURR " << path_it->string() << "\n";
    prev_path = path_it;
  }
}

void help()
{
  std::cout <<
    "--input-dir <i>:  Set input directory containing RGB images\n"
    "--output-dir <o>: Set output directory for annotated images\n"
    "--match <m>:      Enable annotations of matching features across frames\n"
    "--help:           Show help\n";
  exit(1);
}

int main(int argc, char ** argv)
{
  std::string input_dir;
  std::string output_dir;
  bool include_feature_matches = false;
  option long_opts[] = {
    {"input-dir", required_argument, nullptr, 'i'},
    {"output-dir", required_argument, nullptr, 'o'},
    {"match", no_argument, nullptr, 'm'},
    {"help", no_argument, nullptr, 'h'},
    {nullptr, no_argument, nullptr, 0}
  };

  while (true) {
    int opt = getopt_long(argc, argv, "i:o:mh", long_opts, nullptr);
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
      case 'm':
        include_feature_matches = true;
        break;
      case 'h':
      case '?':
      default:
        help();
        break;
    }
  }

  FeaturePlot plotter(input_dir, output_dir);
  if (include_feature_matches) {
    plotter.PlotMatches();
  } else {
    plotter.PlotFeatures();
  }
  return 0;
}
