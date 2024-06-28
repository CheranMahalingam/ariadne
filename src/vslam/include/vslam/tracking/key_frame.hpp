#ifndef VSLAM__KEY_FRAME_HPP_
#define VSLAM__KEY_FRAME_HPP_

#include "vslam/tracking/frame.hpp"

#include "DBoW3/DBoW3.h"

#include <memory>

namespace vslam
{

class KeyFrame
{
public:
  static long int key_frame_id;

  KeyFrame(std::shared_ptr<Frame> frame);

  const DBoW3::FeatureVector & GetFeatures() const;

  const cv::Mat & GetDescriptor(int idx) const;
  const cv::KeyPoint & GetPoint(int idx) const;

  long int curr_id;

private:
  std::shared_ptr<Frame> frame_;
};

}  // vslam

#endif  // VSLAM__KEY_FRAME_HPP_
