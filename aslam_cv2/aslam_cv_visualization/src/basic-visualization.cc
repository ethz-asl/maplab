#include <algorithm>
#include <aslam/matcher/match-helpers.h>

#include "aslam/visualization/basic-visualization.h"

namespace aslam_cv_visualization {

void drawKeypoints(
    const aslam::VisualFrame& frame, cv::Mat* image, int descriptor_type) {
  CHECK_NOTNULL(image);
  if (!frame.getNumKeypointMeasurementsOfType(descriptor_type)) {
    return;
  }

  cv::cvtColor(frame.getRawImage(), *image, cv::COLOR_GRAY2BGR);
  CHECK_NOTNULL(image->data);

  const Eigen::Block<const Eigen::Matrix2Xd> keypoints =
      frame.getKeypointMeasurementsOfType(descriptor_type);
  for (int i = 0; i < keypoints.cols(); ++i) {
    cv::circle(
        *image, cv::Point(keypoints(0, i), keypoints(1, i)), 1,
        cv::Scalar(0, 255, 255), 1, CV_AA);
  }
}

void drawKeypointMatches(
    const aslam::VisualFrame& frame_kp1, const aslam::VisualFrame& frame_k,
    const aslam::FrameToFrameMatches& matches_kp1_k, int descriptor_type,
    cv::Mat* image) {
  CHECK_NOTNULL(image);

  // Convert the original image to grayscale
  cv::cvtColor(frame_kp1.getRawImage(), *image, cv::COLOR_GRAY2BGR);
  CHECK_NOTNULL(image->data);

  const cv::Scalar line_color = cv::Scalar(255, 0, 255);
  const cv::Scalar keypoint_color = cv::Scalar(255, 255, 0);

  const Eigen::Block<const Eigen::Matrix2Xd> keypoints_k =
      frame_k.getKeypointMeasurementsOfType(descriptor_type);
  const Eigen::Block<const Eigen::Matrix2Xd> keypoints_kp1 =
      frame_kp1.getKeypointMeasurementsOfType(descriptor_type);

  for (const aslam::FrameToFrameMatch& match_kp1_k : matches_kp1_k) {
    const int index_kp1 = match_kp1_k.first;
    const int index_k = match_kp1_k.second;

    CHECK_LT(index_kp1, keypoints_kp1.cols());
    CHECK_LT(index_k, keypoints_k.cols());

    cv::circle(
        *image,
        cv::Point(keypoints_kp1(0, index_kp1), keypoints_kp1(1, index_kp1)), 4,
        keypoint_color);
    cv::line(
        *image, cv::Point(keypoints_k(0, index_k), keypoints_k(1, index_k)),
        cv::Point(keypoints_kp1(0, index_kp1), keypoints_kp1(1, index_kp1)),
        line_color);
  }
}

}  // namespace aslam_cv_visualization
