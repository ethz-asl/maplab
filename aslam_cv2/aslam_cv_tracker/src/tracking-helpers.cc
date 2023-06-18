#include "aslam/tracker/tracking-helpers.h"

#include <aslam/common/memory.h>
#include <aslam/common/pose-types.h>
#include <aslam/frames/visual-frame.h>
#include <glog/logging.h>
#include <Eigen/Core>
#include <opencv2/core/core.hpp>

namespace aslam {

void insertCvKeypointsAndDescriptorsIntoEmptyVisualFrame(
    const std::vector<cv::KeyPoint>& new_cv_keypoints, const cv::Mat& new_cv_descriptors,
    const double fixed_keypoint_uncertainty_px, aslam::VisualFrame* frame) {
  CHECK_NOTNULL(frame);
  CHECK(!frame->hasKeypointMeasurements() || frame->getNumKeypointMeasurements() == 0u);
  CHECK(!frame->hasDescriptors() || frame->getDescriptors().cols() == 0);
  CHECK(!frame->hasKeypointMeasurementUncertainties() ||
        frame->getKeypointMeasurementUncertainties().rows() == 0);
  CHECK(!frame->hasKeypointOrientations() || frame->getKeypointOrientations().rows() == 0);
  CHECK(!frame->hasKeypointScales() || frame->getKeypointScales().rows() == 0);
  CHECK(!frame->hasKeypointScores() || frame->getKeypointScores().rows() == 0);
  CHECK(!frame->hasTrackIds() || frame->getTrackIds().rows() == 0);
  CHECK_GT(fixed_keypoint_uncertainty_px, 0.0);
  CHECK_EQ(new_cv_keypoints.size(), static_cast<size_t>(new_cv_descriptors.rows));
  CHECK_EQ(new_cv_descriptors.type(), CV_8UC1);
  CHECK(new_cv_descriptors.isContinuous());

  const size_t num_new_keypoints = new_cv_keypoints.size();

  Eigen::Matrix2Xd new_keypoints_measurements;
  Eigen::VectorXd new_keypoint_scores;
  Eigen::VectorXd new_keypoint_scales;
  Eigen::VectorXd new_keypoint_orientations;
  new_keypoints_measurements.resize(Eigen::NoChange, num_new_keypoints);
  new_keypoint_scores.resize(num_new_keypoints);
  new_keypoint_scales.resize(num_new_keypoints);
  new_keypoint_orientations.resize(num_new_keypoints);
  for (size_t idx = 0u; idx < num_new_keypoints; ++idx) {
    const cv::KeyPoint& cv_keypoint = new_cv_keypoints[idx];
    new_keypoints_measurements.col(idx)(0) = static_cast<double>(cv_keypoint.pt.x);
    new_keypoints_measurements.col(idx)(1) = static_cast<double>(cv_keypoint.pt.y);
    new_keypoint_scores(idx) = static_cast<double>(cv_keypoint.response);
    new_keypoint_scales(idx) = static_cast<double>(cv_keypoint.size);
    new_keypoint_orientations(idx) = static_cast<double>(cv_keypoint.angle);
  }

  frame->swapKeypointMeasurements(&new_keypoints_measurements);
  frame->swapKeypointScores(&new_keypoint_scores);
  frame->swapKeypointScales(&new_keypoint_scales);
  frame->swapKeypointOrientations(&new_keypoint_orientations);

  Eigen::VectorXd uncertainties(num_new_keypoints);
  uncertainties.setConstant(fixed_keypoint_uncertainty_px);
  frame->swapKeypointMeasurementUncertainties(&uncertainties);

  // Set invalid track ids.
  Eigen::VectorXi track_ids(num_new_keypoints);
  track_ids.setConstant(-1);
  frame->swapTrackIds(&track_ids);

  frame->setDescriptors(
      // Switch cols/rows as Eigen is col-major and cv::Mat is row-major.
      Eigen::Map<const aslam::VisualFrame::DescriptorsT>(
          new_cv_descriptors.data, new_cv_descriptors.cols, new_cv_descriptors.rows));
}

void insertAdditionalCvKeypointsAndDescriptorsToVisualFrame(
    const std::vector<cv::KeyPoint>& new_cv_keypoints, const cv::Mat& new_cv_descriptors,
    const double fixed_keypoint_uncertainty_px, aslam::VisualFrame* frame) {
  CHECK_NOTNULL(frame);
  CHECK(frame->hasKeypointMeasurements());
  CHECK(frame->hasDescriptors());
  CHECK(frame->hasKeypointMeasurementUncertainties());
  CHECK(frame->hasKeypointOrientations());
  CHECK(frame->hasKeypointScales());
  CHECK(frame->hasKeypointScores());
  CHECK(frame->hasTrackIds());
  CHECK_GT(fixed_keypoint_uncertainty_px, 0.0);
  CHECK_EQ(new_cv_keypoints.size(), static_cast<size_t>(new_cv_descriptors.rows));
  CHECK_EQ(new_cv_descriptors.type(), CV_8UC1);
  CHECK(new_cv_descriptors.isContinuous());

  const size_t kInitialSize = frame->getTrackIds().size();
  const size_t kAdditionalSize = new_cv_keypoints.size();
  const size_t extended_size = kInitialSize + kAdditionalSize;

  Eigen::Matrix2Xd* const keypoint_measurements =
      frame->getKeypointMeasurementsMutable();
  Eigen::VectorXd* const keypoint_orientations =
      frame->getKeypointOrientationsMutable();
  Eigen::VectorXd* const keypoint_scales =
      frame->getKeypointScalesMutable();
  Eigen::VectorXd* const keypointi_scores =
      frame->getKeypointScoresMutable();
  Eigen::VectorXi* const track_ids = frame->getTrackIdsMutable();
  Eigen::VectorXd* const keypoint_uncertainties =
      frame->getKeypointMeasurementUncertaintiesMutable();
  VisualFrame::DescriptorsT* const descriptors =
      frame->getDescriptorsMutable();

  keypoint_measurements->conservativeResize(
      Eigen::NoChange, extended_size);
  keypoint_orientations->conservativeResize(extended_size);
  keypoint_scales->conservativeResize(extended_size);
  keypointi_scores->conservativeResize(extended_size);
  track_ids->conservativeResize(extended_size);
  keypoint_uncertainties->conservativeResize(extended_size);
  descriptors->conservativeResize(Eigen::NoChange, extended_size);

  Eigen::Matrix2Xd new_keypoint_measurements(2, kAdditionalSize);
  Eigen::VectorXd new_keypoint_orientations(kAdditionalSize);
  Eigen::VectorXd new_keypoint_scales(kAdditionalSize);
  Eigen::VectorXd new_keypoint_scores(kAdditionalSize);
  for (size_t i = 0u; i < kAdditionalSize; ++i) {
    const cv::KeyPoint keypoint = new_cv_keypoints[i];
    new_keypoint_measurements(0, i) = static_cast<double>(keypoint.pt.x);
    new_keypoint_measurements(1, i) = static_cast<double>(keypoint.pt.y);
    new_keypoint_orientations(i) = static_cast<double>(keypoint.angle);
    new_keypoint_scales(i) = static_cast<double>(keypoint.size);
    new_keypoint_scores(i) = static_cast<double>(keypoint.response);
  }

  keypoint_measurements->block(0, kInitialSize, 2,kAdditionalSize) =
          new_keypoint_measurements;
  keypoint_orientations->segment(kInitialSize, kAdditionalSize) =
          new_keypoint_orientations;
  keypoint_scales->segment(kInitialSize, kAdditionalSize) =
          new_keypoint_scales;
  keypointi_scores->segment(kInitialSize, kAdditionalSize) =
          new_keypoint_scores;
  track_ids->segment(kInitialSize, kAdditionalSize).setConstant(-1);
  keypoint_uncertainties->segment(kInitialSize, kAdditionalSize)
          .setConstant(fixed_keypoint_uncertainty_px);
  descriptors->block(0, kInitialSize, new_cv_descriptors.cols,
                     new_cv_descriptors.rows) =
      Eigen::Map<VisualFrame::DescriptorsT>(
          // Switch cols/rows as Eigen is col-major and cv::Mat is row-major.
          new_cv_descriptors.data, new_cv_descriptors.cols,
          new_cv_descriptors.rows);
}

}  // namespace aslam
