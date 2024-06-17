#include "cuda_vslam_ros/tools/tum_bag_generator.hpp"

#include <cv_bridge/cv_bridge.h>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <rosbag2_cpp/writer.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <fstream>
#include <memory>

TUMBagGenerator::TUMBagGenerator(std::string base_path)
: input_path_(base_path)
{
}

void TUMBagGenerator::Generate(std::string output_dir)
{
  loadImages("rgb.txt", "depth.txt");

  std::filesystem::remove_all(std::filesystem::path(output_dir));
  auto bag_dir = output_dir;
  rosbag2_cpp::Writer writer;
  rosbag2_storage::StorageOptions opts;
  opts.uri = bag_dir;
  writer.open(opts);

  for (auto [timestamp, event]:events_) {
    auto [type, path] = event;
    auto image_path = std::filesystem::path(input_path_).append(path);
    auto image = cv::imread(image_path.string(), cv::IMREAD_UNCHANGED);
    sensor_msgs::msg::Image msg;
    if (type == ImageType::RGB) {
      cv_bridge::CvImage(std_msgs::msg::Header(), "rgb8", image).toImageMsg(msg);
    } else {
      cv_bridge::CvImage(std_msgs::msg::Header(), "mono16", image).toImageMsg(msg);
    }
    auto ros_time = rclcpp::Time(static_cast<int64_t>(timestamp * 1e9));
    msg.header.frame_id = "base_link";
    msg.header.stamp = ros_time;
    if (type == ImageType::RGB) {
      writer.write(msg, "/sensing/camera/rgb", ros_time);
    } else {
      writer.write(msg, "/sensing/camera/depth", ros_time);
    }
  }
}

void TUMBagGenerator::loadImages(
  std::string rgb_filename, std::string depth_filename)
{
  auto rgb_path = std::filesystem::path(input_path_).append(rgb_filename);
  auto depth_path = std::filesystem::path(input_path_).append(depth_filename);
  readTimestamps(rgb_path, ImageType::RGB);
  readTimestamps(depth_path, ImageType::DEPTH);
}

void TUMBagGenerator::readTimestamps(
  std::filesystem::path file_path, ImageType type)
{
  std::ifstream f;
  f.open(file_path.string());
  std::string line;
  for (int i = 0; std::getline(f, line); i++) {
    if (i < 3) {
      continue;
    }
    double timestamp = std::stod(line.substr(0, line.find(" ")));
    std::string path = line.substr(line.find(" ") + 1);
    events_.insert({timestamp, std::pair(type, path)});
  }
}

int main(int argc, char ** argv)
{
  auto generator = TUMBagGenerator("data/rgbd_dataset_freiburg2_pioneer_slam");
  generator.Generate("data/rosbags/rgbd_dataset_freiburg2_pioneer_slam");
  return 0;
}
