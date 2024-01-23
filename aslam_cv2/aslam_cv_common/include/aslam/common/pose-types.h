#ifndef ASLAM_COMMON_POSE_TYPES_H_
#define ASLAM_COMMON_POSE_TYPES_H_

#include <vector>

#include <aslam/common/memory.h>

#include <Eigen/Core>
#include <kindr/minimal/position.h>
#include <kindr/minimal/quat-transformation.h>
#include <kindr/minimal/rotation-quaternion.h>

namespace aslam {

typedef kindr::minimal::QuatTransformation Transformation;
typedef Aligned<std::vector, Transformation> TransformationVector;
typedef Eigen::Matrix<double, 6, 6> TransformationCovariance;
typedef Aligned<std::vector, TransformationCovariance>
    TransformationCovarianceList;
typedef kindr::minimal::RotationQuaternion Quaternion;
typedef kindr::minimal::AngleAxis AngleAxis;
typedef kindr::minimal::Position Position3D;

// Types used in ceres error terms, where templating for Jet types
// is necessary.
template <class Scalar>
using QuaternionTemplate = kindr::minimal::RotationQuaternionTemplate<Scalar>;

template <class Scalar>
using PositionTemplate = kindr::minimal::PositionTemplate<Scalar>;

}  // namespace aslam

#endif  // ASLAM_COMMON_POSE_TYPES_H_
