#ifndef ASLAM_CV_DETECTORS_LSD
#define ASLAM_CV_DETECTORS_LSD

#include <memory>

#include <aslam/common/macros.h>
#include <Eigen/Core>
#include <lsd/lsd-opencv.h>

#include "aslam/detectors/line.h"

namespace aslam {

class LineSegmentDetector {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  struct Options {
    size_t min_segment_length_px;
    Options() :
      min_segment_length_px(20u) {};
  };

  LineSegmentDetector(const Options& options);
  ~LineSegmentDetector();

  void detect(const cv::Mat& image, Lines* lines);

  /// Draw a list of lines onto a color(!) image.
  void drawLines(const Lines& lines, cv::Mat* image);

 private:
  cv::Ptr<aslamcv::LineSegmentDetector> line_detector_;
  const Options options_;
};

}  // namespace aslam

#endif  // ASLAM_CV_DETECTORS_LSD
