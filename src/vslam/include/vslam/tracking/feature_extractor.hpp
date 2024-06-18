#ifndef VSLAM__FEATURE_EXTRACTOR_HPP_
#define VSLAM__FEATURE_EXTRACTOR_HPP_

#include <opencv2/core.hpp>

#include <vector>

namespace vslam
{

class FeatureExtractor
{
public:
  static constexpr int PATCH_SIZE = 31;
  static constexpr int EDGE_THRESHOLD = 19;
  static constexpr int KEY_POINT_CELL_LEN = 30;
  static constexpr int FAST_DETECTOR_RADIUS = 3;
  static constexpr int GAUSSIAN_KERNEL_SIGMA = 2;

  FeatureExtractor(
    int orb_levels, float orb_scale_factor, int orb_num_features,
    int orb_initial_FAST_threshold, int orb_min_FAST_threshold);

  void ComputeFeatures(
    cv::InputArray image_arr, std::vector<cv::KeyPoint> & key_points,
    cv::OutputArray descriptors);

private:
  struct RectBounds
  {
    int x_min;
    int x_max;
    int y_min;
    int y_max;

    RectBounds(int x_min, int x_max, int y_min, int y_max)
    : x_min(x_min), x_max(x_max), y_min(y_min), y_max(y_max) {}
  };

  struct Grid
  {
    int cols;
    int rows;
    int cell_width;
    int cell_height;

    Grid(int cols, int rows, int width, int height)
    : cols(cols), rows(rows), cell_width(width), cell_height(height) {}
  };

  struct QuadTreeNode
  {
    RectBounds bounds;
    std::vector<cv::KeyPoint> children;

    QuadTreeNode()
    : bounds(0, 0, 0, 0) {}
    QuadTreeNode(
      RectBounds bounds, const std::vector<cv::KeyPoint> & children)
    : bounds(bounds), children(children) {}
    void Split(
      struct QuadTreeNode & n1, struct QuadTreeNode & n2,
      struct QuadTreeNode & n3, struct QuadTreeNode & n4);
  };

  std::vector<std::vector<cv::KeyPoint>> computeKeyPoints(
    const std::vector<cv::Mat> & pyramid);

  void computeDescriptors(
    const std::vector<cv::Mat> & pyramid,
    std::vector<std::vector<cv::KeyPoint>> & key_points,
    cv::Mat & descriptors);

  void computeOrientation(
    const std::vector<cv::Mat> & pyramid,
    std::vector<std::vector<cv::KeyPoint>> & key_points);

  std::vector<cv::Mat> buildImagePyramid(const cv::Mat & image);

  std::vector<cv::KeyPoint> buildQuadTree(
    const cv::Mat & image, int num_features, const RectBounds & bounds,
    const std::vector<cv::KeyPoint> & key_points);

  std::vector<cv::KeyPoint> computeFASTFeatures(
    const cv::Mat & image, const RectBounds & bounds, const Grid & image_grid);

  void computeSteeredBRIEF(
    const cv::Mat & image, const cv::KeyPoint & point,
    unsigned char * descriptor);

  cv::KeyPoint findMaxResponse(const QuadTreeNode & node);

  float intensityCentroidAngle(
    const cv::Mat & image, const cv::Point2f & point);

  // Tunable ORB keypoint detection parameters.
  int orb_levels_;
  float orb_scale_factor_;
  int orb_num_features_;
  int orb_initial_FAST_threshold_;
  int orb_min_FAST_threshold_;

  // Factor by which the scale of each image has been reduced in the pyramid.
  std::vector<float> image_pyramid_scale_;
  // Target number of keypoints to detect in each level of the pyramid.
  std::vector<int> features_per_level_;
  // Number of pixels away from center in each row of a circular patch.
  std::vector<int> chord_size_;
};

}  // vslam

#endif  // VSLAM__FEATURE_EXTRACTOR_HPP_

