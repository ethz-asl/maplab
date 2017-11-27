#ifndef MAPLAB_COMMON_POSE_TYPES_H_
#define MAPLAB_COMMON_POSE_TYPES_H_

#include <aslam/common/memory.h>
#include <kindr/minimal/position.h>
#include <kindr/minimal/quat-transformation.h>
#include <kindr/minimal/rotation-quaternion.h>

namespace pose {

typedef kindr::minimal::QuatTransformation Transformation;
typedef kindr::minimal::RotationQuaternion Quaternion;
typedef kindr::minimal::Position Position3D;
typedef Aligned<std::vector, Position3D> Position3DVector;

// Types used in ceres error terms, where templating for Jet types
// is necessary.
template <class Scalar>
using QuaternionTemplate = kindr::minimal::RotationQuaternionTemplate<Scalar>;

template <class Scalar>
using PositionTemplate = kindr::minimal::PositionTemplate<Scalar>;

template <class Scalar>
using TransformationTemplate =
    kindr::minimal::QuatTransformationTemplate<Scalar>;

}  // namespace pose

#endif  // MAPLAB_COMMON_POSE_TYPES_H_
