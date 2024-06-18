#ifndef VSLAM__TUM_ROSBAG_HPP_
#define VSLAM__TUM_ROSBAG_HPP_

#include <rclcpp/rclcpp.hpp>

#include <filesystem>
#include <map>

class TUMRosbag
{
public:
  enum class ImageType
  {
    RGB,
    DEPTH,
  };

  TUMRosbag(std::string base_path);

  void Generate(std::string output_dir);

private:
  void loadImages(
    std::string rgb_filename, std::string depth_filename);
  void readTimestamps(
    std::filesystem::path file_path, ImageType type);

  std::string input_path_;
  std::multimap<double, std::pair<ImageType, std::string>> events_;
};

#endif  // VSLAM__TUM_ROSBAG_HPP_
