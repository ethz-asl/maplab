#ifndef MAPLAB_COMMON_INTERPOLATION_HELPERS_H_
#define MAPLAB_COMMON_INTERPOLATION_HELPERS_H_

#include <aslam/common/pose-types.h>

namespace common {

template <typename Time, typename InterpolateType>
void linearInterpolation(
    const Time t1, const InterpolateType& x1, const Time t2,
    const InterpolateType& x2, const Time t_interpolated,
    InterpolateType* x_interpolated) {
  CHECK_NOTNULL(x_interpolated);
  CHECK_LE(t1, t2);
  CHECK_LE(t1, t_interpolated);
  CHECK_LE(t_interpolated, t2);
  if (t1 == t2) {
    CHECK_EQ(x1, x2);
    *x_interpolated = x1;
    return;
  }
  *x_interpolated = x1 + (x2 - x1) / (t2 - t1) * (t_interpolated - t1);
}

template <typename Time>
void interpolateRotation(
    const Time t1, const aslam::Quaternion& q_A_B1, const Time t2,
    const aslam::Quaternion& q_A_B2, const Time t_interpolated,
    aslam::Quaternion* q_A_B_interpolated) {
  CHECK_NOTNULL(q_A_B_interpolated);
  CHECK_LT(t1, t2);
  CHECK_LE(t1, t_interpolated);
  CHECK_LE(t_interpolated, t2);

  q_A_B_interpolated->setIdentity();
  const aslam::Quaternion q_B1_B2 = q_A_B1.inverse() * q_A_B2;
  const double theta = (t_interpolated - t1) / static_cast<double>(t2 - t1);
  *q_A_B_interpolated = q_A_B1 * aslam::Quaternion::exp(theta * q_B1_B2.log());
}

template <typename Time>
void interpolateTransformation(
    const Time t1, const aslam::Transformation& T_A_B1, const Time t2,
    const aslam::Transformation& T_A_B2, const Time t_interpolated,
    aslam::Transformation* T_A_B_interpolated) {
  CHECK_NOTNULL(T_A_B_interpolated);
  CHECK_LT(t1, t2);
  CHECK_LE(t1, t_interpolated);
  CHECK_LE(t_interpolated, t2);

  aslam::Position3D A_p_AB_interpolated;
  common::linearInterpolation(
      t1, T_A_B1.getPosition(), t2, T_A_B2.getPosition(), t_interpolated,
      &A_p_AB_interpolated);

  aslam::Quaternion q_A_B_interpolated;
  common::interpolateRotation(
      t1, T_A_B1.getRotation(), t2, T_A_B2.getRotation(), t_interpolated,
      &q_A_B_interpolated);
  *T_A_B_interpolated =
      aslam::Transformation(q_A_B_interpolated, A_p_AB_interpolated);
}

template <typename Time, typename InterpolateType>
struct LinearInterpolationFunctor {
  void operator()(
      const Time t1, const InterpolateType& x1, const Time t2,
      const InterpolateType& x2, const Time t_interpolated,
      InterpolateType* x_interpolated) {
    linearInterpolation(t1, x1, t2, x2, t_interpolated, x_interpolated);
  }
};

template <typename Time>
struct LinearInterpolationFunctor<Time, aslam::Quaternion> {
  void operator()(
      const Time t1, const aslam::Quaternion& x1, const Time t2,
      const aslam::Quaternion& x2, const Time t_interpolated,
      aslam::Quaternion* x_interpolated) {
    interpolateRotation(t1, x1, t2, x2, t_interpolated, x_interpolated);
  }
};

template <typename Time>
struct LinearInterpolationFunctor<Time, aslam::Transformation> {
  void operator()(
      const Time t1, const aslam::Transformation& x1, const Time t2,
      const aslam::Transformation& x2, const Time t_interpolated,
      aslam::Transformation* x_interpolated) {
    interpolateTransformation(t1, x1, t2, x2, t_interpolated, x_interpolated);
  }
};

}  // namespace common

#endif  // MAPLAB_COMMON_INTERPOLATION_HELPERS_H_
