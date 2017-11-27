#ifndef GEOMETRIC_VISION_LINEAR_TRIANGULATION_H_
#define GEOMETRIC_VISION_LINEAR_TRIANGULATION_H_

#include <vector>

#include <Eigen/Core>

#include <aslam/common/memory.h>
#include <maplab-common/pose_types.h>

namespace geometric_vision {

class LinearTriangulation {
 public:
  bool triangulateFromNormalizedTwoViews(
      const Eigen::Vector2d& measurement1,
      const pose::Transformation& camera_pose1,
      const Eigen::Vector2d& measurement2,
      const pose::Transformation& camera_pose2,
      Eigen::Vector3d* triangulated_point);

  bool triangulateFromNormalizedTwoViewsHomogeneous(
      const Eigen::Vector2d& measurement1,
      const pose::Transformation& camera_pose1,
      const Eigen::Vector2d& measurement2,
      const pose::Transformation& camera_pose2,
      Eigen::Vector3d* triangulated_point);

  bool triangulateFromNormalizedNViews(
      const Aligned<std::vector, Eigen::Vector2d>& measurements,
      const Aligned<std::vector, pose::Transformation>& G_T_C,
      Eigen::Vector3d* triangulated_point);
};

}  // namespace geometric_vision

#endif  // GEOMETRIC_VISION_LINEAR_TRIANGULATION_H_
