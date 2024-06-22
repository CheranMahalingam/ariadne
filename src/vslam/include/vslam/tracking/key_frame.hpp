#ifndef VSLAM__KEY_FRAME_HPP_
#define VSLAM__KEY_FRAME_HPP_

#include "vslam/tracking/frame.hpp"

namespace vslam
{

class KeyFrame
{
public:
  static long int key_frame_id;

  KeyFrame(Frame & frame);

private:
  long int curr_id_;
};

}  // vslam

#endif  // VSLAM__KEY_FRAME_HPP_
