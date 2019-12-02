#include "localization-fusion/filter-utilities.h"
#include "localization-fusion/filter-common.h"

#include <string>
#include <vector>

namespace maplab {

double clampRotation(double rotation) {
  CHECK(!std::isnan(rotation));
  CHECK(!std::isinf(rotation));
  rotation = fmod(rotation + M_PI, localization_fusion::TAU);
  if (rotation < 0)
    rotation += localization_fusion::TAU;
  return rotation - M_PI;
}

void RPYtoQuaternion(
    double r, double p, double y,
    Eigen::Quaterniond& q) {  // NOLINT
  double halfYaw = y * 0.5f;
  double halfPitch = p * 0.5f;
  double halfRoll = r * 0.5f;
  double cosYaw = cos(halfYaw);
  double sinYaw = sin(halfYaw);
  double cosPitch = cos(halfPitch);
  double sinPitch = sin(halfPitch);
  double cosRoll = cos(halfRoll);
  double sinRoll = sin(halfRoll);
  q.x() = sinRoll * cosPitch * cosYaw - cosRoll * sinPitch * sinYaw;
  q.y() = cosRoll * sinPitch * cosYaw + sinRoll * cosPitch * sinYaw;
  q.z() = cosRoll * cosPitch * sinYaw - sinRoll * sinPitch * cosYaw;
  q.w() = cosRoll * cosPitch * cosYaw + sinRoll * sinPitch * sinYaw;
}

void matrixToRPY(
    const Eigen::Matrix3d& m, double& yaw,  // NOLINT
    double& pitch,                          // NOLINT
    double& roll) {                         // NOLINT
  // Check that pitch is not at a singularity
  if (abs(m(2, 0)) >= 1) {
    yaw = 0;

    // From difference of angles formula
    if (m(2, 0) < 0) {
      // gimbal locked down
      double delta = atan2(m(0, 1), m(0, 2));
      pitch = M_PI / 2.0;
      roll = delta;
    } else {
      // gimbal locked up
      double delta = atan2(-m(0, 1), -m(0, 2));
      pitch = -1.0 * M_PI / 2.0;
      roll = delta;
    }
  } else {
    pitch = -1.0 * asin(m(2, 0));

    roll = atan2(m(2, 1) / cos(pitch), m(2, 2) / cos(pitch));

    yaw = atan2(m(1, 0) / cos(pitch), m(0, 0) / cos(pitch));
  }
}

}  // namespace maplab
