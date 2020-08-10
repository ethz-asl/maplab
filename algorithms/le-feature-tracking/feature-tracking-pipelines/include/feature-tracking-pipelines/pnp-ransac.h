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

bool PerformTemporalFrameToFrameRansac(
    const aslam::Camera& camera, const KeyframeFeatures& keyframe_features_kp1,
    const KeyframeFeatures& keyframe_features_k, const RansacSettings& settings,
    const aslam::Quaternion& q_Ckp1_Ck,
    aslam::FrameToFrameMatches* inlier_matches_kp1_k,
    aslam::FrameToFrameMatches* outlier_matches_kp1_k);

bool PerformTemporalFrameToFrameRansac(
    pcl::PointCloud<pcl::PointXYZI>::ConstPtr cloud,
    const KeyframeFeatures& keyframe_features_kp1,
    const KeyframeFeatures& keyframe_features_k,
    Eigen::Matrix3d* best_rotation_matrix, Eigen::Vector3d* best_translation,
    std::vector<size_t>* best_inliers, std::vector<size_t>* best_outliers);

// Based on two sets of 3D points, this function calculates a matching translation,
// rotation, outliers, inliers with RANSAC 
void RansacTransformationFor3DPoints(
    std::vector<Eigen::Vector3d> point_set_1,
    std::vector<Eigen::Vector3d> point_set_2, Eigen::Matrix3d* rotation_matrix,
    Eigen::Vector3d* translation, std::vector<size_t>* best_inliers,
    std::vector<size_t>* best_outliers, double ransac_threshold, int ransac_max_iterations);

bool backProject3d(
    pcl::PointCloud<pcl::PointXYZI>::ConstPtr cloud, const cv::Mat* image,
    const Eigen::Ref<const Eigen::Vector2d>& keypoint,
    Eigen::Vector3d* out_point_3d);

bool backProject3d(
    pcl::PointCloud<pcl::PointXYZI>::ConstPtr cloud,
    const Eigen::Ref<const Eigen::Vector2d>& keypoint,
    Eigen::Vector3d* out_point_3d);

}  // namespace feature_tracking_pipelines

#endif  // FEATURE_TRACKING_PIPELINES_PNP_RANSAC_H_i
