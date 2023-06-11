#include "aslam/detectors/line-segment-detector.h"

#include <memory>

#include <Eigen/Core>
#include <lsd/lsd-opencv.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

namespace aslam {

LineSegmentDetector::LineSegmentDetector(const Options& options)
    : options_(options) {
  line_detector_ = aslamcv::createLineSegmentDetectorPtr(
      cv::LSD_REFINE_STD, 0.8, 0.6, 2.0, 16.5, 0.0, 0.65, 1024);
}

LineSegmentDetector::~LineSegmentDetector() {}

void LineSegmentDetector::detect(const cv::Mat& image, Lines* lines) {
  CHECK_NOTNULL(lines)->clear();
  CHECK(!line_detector_.empty());

  std::vector<cv::Vec4i> raw_lines;
  line_detector_->detect(image, raw_lines);
  line_detector_->filterSize(raw_lines, raw_lines, options_.min_segment_length_px);

  lines->reserve(raw_lines.size());
  for (size_t line_idx = 0u; line_idx < raw_lines.size(); ++line_idx) {
    lines->emplace_back(static_cast<double>(raw_lines[line_idx](0)),
                        static_cast<double>(raw_lines[line_idx](1)),
                        static_cast<double>(raw_lines[line_idx](2)),
                        static_cast<double>(raw_lines[line_idx](3)));
  }
}

void LineSegmentDetector::drawLines(const Lines& lines, cv::Mat* image) {
  CHECK_NOTNULL(image);
  CHECK_EQ(image->channels(), 3) << "Color image required.";

  cv::RNG rng(12345);
  for (size_t idx = 0u; idx < lines.size(); ++idx) {
    cv::Point2d start_point(lines[idx].start_point(0), lines[idx].start_point(1));
    cv::Point2d end_point(lines[idx].end_point(0), lines[idx].end_point(1));

    cv::Scalar color(rng.uniform(0,255), rng.uniform(0, 255), rng.uniform(0, 255));
    cv::line(*image, start_point, end_point, color, 2, CV_AA);
  }
}

}  // namespace aslam
