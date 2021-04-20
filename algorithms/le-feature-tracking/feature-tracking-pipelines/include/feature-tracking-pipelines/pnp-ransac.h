#ifndef FEATURE_TRACKING_PIPELINES_PNP_RANSAC_H_
#define FEATURE_TRACKING_PIPELINES_PNP_RANSAC_H_

#include <string>
#include <unordered_set>
#include <vector>

#include <stdlib.h>

#include <Eigen/Core>
#include <aslam/cameras/ncamera.h>
#include <aslam/common/pose-types.h>
#include <aslam/frames/visual-frame.h>
#include <aslam/geometric-vision/match-outlier-rejection-twopt.h>
#include <aslam/matcher/match-helpers.h>
#include <maplab-common/conversions.h>
#include <sensors/lidar.h>

#include "feature-tracking-pipelines/flags.h"
#include "feature-tracking-pipelines/keyframe-features.h"

namespace feature_tracking_pipelines {
struct RansacSettings {
  double ransac_threshold = 1.0 - cos(0.5 * kDegToRad);
  size_t ransac_max_iterations = 200;
  bool fix_random_seed = false;
};

RansacSettings InitRansacSettingsFromGFlags();

class PnpRansac {
 public:
  explicit PnpRansac(const std::size_t ring_size, const std::size_t beam_size);

  bool performTemporalFrameToFrameRansac(
      const aslam::Camera& camera,
      const KeyframeFeatures& keyframe_features_kp1,
      const KeyframeFeatures& keyframe_features_k,
      const RansacSettings& settings, const aslam::Quaternion& q_Ckp1_Ck,
      aslam::FrameToFrameMatches* inlier_matches_kp1_k,
      aslam::FrameToFrameMatches* outlier_matches_kp1_k);

  bool performTemporalFrameToFrameRansac(
      pcl::PointCloud<pcl::PointXYZI>::ConstPtr cloud,
      const KeyframeFeatures& keyframe_features_kp1,
      const KeyframeFeatures& keyframe_features_k,
      Eigen::Matrix3d* best_rotation_matrix, Eigen::Vector3d* best_translation,
      std::vector<size_t>* best_inliers, std::vector<size_t>* best_outliers);

  // Based on two sets of 3D points, this function calculates a matching
  // translation, rotation, outliers, inliers with RANSAC
  void ransacTransformationFor3DPoints(
      const std::vector<Eigen::Vector3d>& point_set_1,
      const std::vector<Eigen::Vector3d>& point_set_2,
      const double ransac_threshold, const std::size_t ransac_max_iterations,
      Eigen::Matrix3d* rotation_matrix, Eigen::Vector3d* translation,
      std::vector<size_t>* best_inliers, std::vector<size_t>* best_outliers);

  bool backProject3dUsingCloud(
      const pcl::PointCloud<pcl::PointXYZI>::ConstPtr& cloud,
      const Eigen::Ref<const Eigen::Vector2d>& keypoint,
      Eigen::Vector3d* out_point_3d) const;

 private:
  const std::size_t height_;
  const std::size_t width_;
  std::vector<std::size_t> px_offset_;
  const std::size_t kPictureStretchingFactor = 4u;

  std::vector<std::size_t> getPxOffset(const std::size_t lidar_mode) const;
};

}  // namespace feature_tracking_pipelines

#endif  // FEATURE_TRACKING_PIPELINES_PNP_RANSAC_H_i
