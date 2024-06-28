#include "vslam/tracking/key_frame.hpp"

namespace vslam
{

long int KeyFrame::key_frame_id = 0;

KeyFrame::KeyFrame(std::shared_ptr<Frame> frame)
: curr_id(key_frame_id), frame_(frame)
{
  key_frame_id++;
}

const DBoW3::FeatureVector & KeyFrame::GetFeatures() const
{
  return frame_->GetFeatures();
}

const cv::Mat & KeyFrame::GetDescriptor(int idx) const
{
  return frame_->GetDescriptor(idx);
}

const cv::KeyPoint & KeyFrame::GetPoint(int idx) const
{
  return frame_->GetPoint(idx);
}

}  // vslam
