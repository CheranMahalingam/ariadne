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

}  // vslam
