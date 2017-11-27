#ifndef GEOMETRIC_VISION_ROTATION_ONLY_DETECTOR_H_
#define GEOMETRIC_VISION_ROTATION_ONLY_DETECTOR_H_

#include <vector>

#include <Eigen/Core>
#include <aslam/cameras/ncamera.h>
#include <aslam/frames/visual-frame.h>
#include <aslam/frames/visual-nframe.h>
#include <aslam/matcher/match.h>
#include <glog/logging.h>

#include "geometric-vision/relative-non-central-pnp.h"

namespace geometric_vision {

// Inspired by:
// "Detecting and dealing with hovering maneuvers in vision-aided inertial
// navigation systems"
// by Kottas, Dimitrios G. and Wu, Kejian J. and Roumeliotis, Stergios I., IROS
// 2013.
class RotationOnlyDetector {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /// \brief Default constructor with default motion classification threshold.
  RotationOnlyDetector() : motion_classification_threshold_(1.0e-5) {
    is_only_rotated_k_ = true;
  }

  /// \brief Constructor to set the motion classification threshold.
  explicit RotationOnlyDetector(double motion_classification_threshold)
      : motion_classification_threshold_(motion_classification_threshold) {
    CHECK_GT(motion_classification_threshold_, 0.0);
    is_only_rotated_k_ = true;
  }

  /// \brief Returns true if the motion detected is rotation-only.
  /// @param[in] nframe_kp1 Contains camera_rig and observations from viewpoint
  /// k+1.
  /// @param[in] nframe_k Contains camera_rig and observations from viewpoint k.
  /// @param[in] matches_kp1_k Matches from camera i in viewpoint k+1 and camera
  /// i in
  ///            viewpoint k.
  /// @param[in] q_Bkp1_Bk Rotation matrix taking points from the frame of
  /// reference at
  ///                      time k to the frame of reference at time k+1.
  /// @param[in] remove_outliers Remove outliers using 2-point RANSAC if set to
  /// true.
  bool isRotationOnlyMotion(
      const aslam::VisualNFrame::Ptr& nframe_kp1,
      const aslam::VisualNFrame::Ptr& nframe_k,
      const aslam::FrameToFrameMatchesList& matches_kp1_k,
      const aslam::Quaternion& q_Bkp1_Bk, const bool remove_outliers);

  /// /brief Return the deviation from perfectly parallel bearing vectors. If
  /// the deviation is
  /// small then the motion could be explained by rotation-only. If the
  /// deviation is large
  /// it is a mixed motion or translation-only motion.
  inline double getDeviationFromParallelBearingVectorLastUpdate() const {
    return deviation_from_parallel_bearing_vector_;
  }

  /// /brief Return the motion classification threshold.
  inline double getMotionClassificationThreshold() const {
    return motion_classification_threshold_;
  }

 private:
  /// \brief Solves 2-point RANSAC problem and returns inlier matches.
  /// @param[in] nframe_kp1 Contains camera_rig and observations from viewpoint
  /// k+1.
  /// @param[in] nframe_k Contains camera_rig and observations from viewpoint k.
  /// @param[in] matches_kp1_k Matches from camera i in viewpoint k+1 and camera
  /// i in
  ///            viewpoint k.
  /// @param[in] q_Ckp1_Ck Rotation matrix taking points from the frame of
  /// reference at
  ///                      time k to the frame of reference at time k+1.
  /// @param[in] camera_index Current camera index.
  /// @param[out] inlier_matches_kp1_k Inliers determined by 2-point RANSAC
  /// problem.
  void getInlierMatches(
      const aslam::VisualNFrame::Ptr& nframe_kp1,
      const aslam::VisualNFrame::Ptr& nframe_k,
      const aslam::FrameToFrameMatchesList& matches_kp1_k,
      const aslam::Quaternion& q_Ckp1_Ck, const size_t camera_index,
      aslam::FrameToFrameMatches* inlier_matches_kp1_k) const;

  /// Classifies the motion in rotation-only and not-rotation-only motion. If
  /// deviation_from_parallel_bearing_vector is below the threshold, then the
  /// motion is
  /// classified as rotation-only.
  double motion_classification_threshold_;
  /// Compare the bearing vector in viewpoint k+1 to the bearing vector rotated
  /// from
  /// viewpoint k to viewpoint k+1, if they are parallel then the deviation in
  /// the
  /// normalized bearing vectors is small.
  double deviation_from_parallel_bearing_vector_;
  /// Stores the classification of the last timestep to avoid flickering.
  bool is_only_rotated_k_;

  /// 2-pt RANSAC threshold used to reject outliers.
  static constexpr double kTwoPointRansacThreshold = 0.000005181;
  /// Max. number of RANSAC iterations used to reject outliers.
  static constexpr size_t kTwoPointRansacMaxIterations = 100;
};

}  // namespace geometric_vision
#endif  // GEOMETRIC_VISION_ROTATION_ONLY_DETECTOR_H_
