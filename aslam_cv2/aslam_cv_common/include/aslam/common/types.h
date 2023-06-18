#ifndef ASLAM_TYPES_H_
#define ASLAM_TYPES_H_

#include <kindr/minimal/quat-transformation.h>
#include <opencv2/imgproc/imgproc.hpp>

namespace aslam {

/// Interpolation method used for image undistortion. (Wrapper for OpenCV types)
enum class InterpolationMethod : int {
  /// A bilinear interpolation (used by default).
  Linear = cv::INTER_LINEAR,
  /// A bicubic interpolation over 4x4 pixel neighborhood.
  Cubic = cv::INTER_CUBIC,
  /// A Lanczos interpolation over 8x8 pixel neighborhood.
  Lanczos = cv::INTER_LANCZOS4,
  /// A nearest-neighbor interpolation.
  NearestNeighbor = cv::INTER_NEAREST,
  /// Resampling using pixel area relation. It may be a prefered method for image decimation,
  /// as it gives moire-free results.  But when the image is zoomed, it is similar to the
  /// INTER_NEAREST method.
  Area = cv::INTER_AREA
};

} // namespace aslam

#endif //ASLAM_TYPES_H_
