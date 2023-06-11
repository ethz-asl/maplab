#include <aslam/pipeline/undistorter.h>

namespace aslam {

Undistorter::Undistorter(Camera::Ptr input_camera, Camera::Ptr output_camera)
    : input_camera_(input_camera),
      output_camera_(output_camera) {}


}  // namespace aslam
