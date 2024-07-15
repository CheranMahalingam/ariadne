#include "vslam/utils.hpp"

namespace vslam
{

void CameraParams::CreateIntrinsics()
{
  k = cv::Mat::eye(3, 3, CV_32F);
  k.at<float>(0, 0) = fx;
  k.at<float>(1, 1) = fy;
  k.at<float>(0, 2) = cx;
  k.at<float>(1, 2) = cy;
}

Pose::Pose()
: initialized(false)
{
}

void Pose::SetPose(cv::Mat pose_cw)
{
  transform_cw = pose_cw;
  rotation_cw = pose_cw.rowRange(0, 3).colRange(0, 3);
  translation_cw = pose_cw.rowRange(0, 3).col(3);

  transform_wc = cv::Mat::eye(4, 4, pose_cw.type());
  rotation_wc = rotation_cw.t();
  translation_wc = -rotation_wc * translation_cw;
  rotation_wc.copyTo(transform_wc.rowRange(0, 3).colRange(0, 3));
  translation_wc.copyTo(transform_wc.rowRange(0, 3).col(3));

  initialized = true;
}

int hamming_distance(const cv::Mat & a, const cv::Mat & b)
{
  const int * a_ptr = a.ptr<int>();
  const int * b_ptr = b.ptr<int>();
  // Compute xor such that all 1s in output denote difference in bits.
  unsigned int v = (*a_ptr) ^ (*b_ptr);
  // https://graphics.stanford.edu/~seander/bithacks.html#CountBitsSetParallel
  v = v - ((v >> 1) & 0x55555555);                    // reuse input as temporary
  v = (v & 0x33333333) + ((v >> 2) & 0x33333333);     // temp
  int dist = (((v + (v >> 4)) & 0xF0F0F0F) * 0x1010101) >> 24; // count
  return dist;
}

g2o::SE3Quat convertToSE3Quat(const cv::Mat & transform)
{
  Eigen::Matrix<double, 3, 3> rotation;
  rotation << (
    transform.at<float>(0, 0), transform.at<float>(0, 1), transform.at<float>(0, 2),
    transform.at<float>(1, 0), transform.at<float>(1, 1), transform.at<float>(1, 2),
    transform.at<float>(2, 0), transform.at<float>(2, 1), transform.at<float>(2, 2));
  Eigen::Matrix<double, 3, 1> translation(
    transform.at<float>(0, 3), transform.at<float>(1, 3), transform.at<float>(2, 3));
  return g2o::SE3Quat(rotation, translation);
}

cv::Mat convertToMat(const g2o::SE3Quat & quat)
{
  auto homogeneous_matrix = quat.to_homogeneous_matrix();
  cv::Mat transform(4, 4, CV_32F);
  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 4; j++) {
      transform.at<float>(i, j) = homogeneous_matrix(i, j);
    }
  }
  return transform;
}

cv::Mat convertToMat(const Eigen::Matrix<double, 3, 1> & matrix)
{
  cv::Mat translation(3, 1, CV_32F);
  for (int i = 0; i < 3; i++) {
    translation.at<float>(i) = matrix(i);
  }
  return translation;
}

Eigen::Matrix<double, 3, 1> convertToVector(const cv::Mat & translation)
{
  auto vec = Eigen::Matrix<double, 3, 1>(
    translation.at<float>(0),
    translation.at<float>(1),
    translation.at<float>(2));
  return vec;
}

}  // vslam
