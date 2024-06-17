#ifndef CUDA_VSLAM_ROS__TUM_BAG_GENERATOR_HPP_
#define CUDA_VSLAM_ROS__TUM_BAG_GENERATOR_HPP_

#include <rclcpp/rclcpp.hpp>

#include <filesystem>
#include <map>

class TUMBagGenerator
{
public:
  enum class ImageType
  {
    RGB,
    DEPTH,
  };

  TUMBagGenerator(std::string base_path);

  void Generate(std::string output_dir);

private:
  void loadImages(
    std::string rgb_filename, std::string depth_filename);
  void readTimestamps(
    std::filesystem::path file_path, ImageType type);

  std::string input_path_;
  std::multimap<double, std::pair<ImageType, std::string>> events_;
};

#endif  // CUDA_VSLAM_ROS__TUM_BAG_GENERATOR_HPP_
