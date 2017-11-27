#ifndef GEOMETRIC_VISION_FIVE_POINT_POSE_ESTIMATOR_H_
#define GEOMETRIC_VISION_FIVE_POINT_POSE_ESTIMATOR_H_

#include <vector>

#include <Eigen/Core>
#include <glog/logging.h>

#include <aslam/cameras/camera-pinhole.h>
#include <aslam/cameras/camera.h>
#include <maplab-common/pose_types.h>
#include <opengv/absolute_pose/methods.hpp>
#include <opengv/sac/Ransac.hpp>
#include <opengv/sac_problems/relative_pose/CentralRelativePoseSacProblem.hpp>

namespace opengv_pose_estimation {

class FivePointPoseEstimator {
 public:
  void ComputePinhole(
      const Eigen::Matrix2Xd& measurements_a,
      const Eigen::Matrix2Xd& measurements_b, double pixel_sigma,
      unsigned int max_ransac_iters, aslam::Camera::ConstPtr camera_ptr,
      pose::Transformation* output_transform, std::vector<int>* inlier_matches);
  void Compute(
      const Eigen::Matrix2Xd& measurements_a,
      const Eigen::Matrix2Xd& measurements_b, double ransac_threshold,
      unsigned int max_ransac_iters, aslam::Camera::ConstPtr camera_ptr,
      pose::Transformation* output_transform, std::vector<int>* inlier_matches);
};

}  // namespace opengv_pose_estimation

#endif  // GEOMETRIC_VISION_FIVE_POINT_POSE_ESTIMATOR_H_
