#include "aslam/calibration/target-base.h"

#include <cmath>
#include <utility>

#include <Eigen/Core>
#include <glog/logging.h>

namespace aslam {
namespace calibration {

TargetBase::TargetBase(
    size_t rows, size_t cols, const Eigen::Matrix3Xd& points_target_frame)
    : rows_(rows), cols_(cols), points_target_frame_(points_target_frame) {
  CHECK_GT(rows, 0u);
  CHECK_GT(cols, 0u);
  CHECK_EQ(static_cast<int>(size()), points_target_frame_.cols());
}

const Eigen::Matrix3Xd& TargetBase::points() const {
  return points_target_frame_;
}

Eigen::Vector3d TargetBase::point(size_t point_idx) const {
  CHECK_LT(point_idx, size());
  return points_target_frame_.col(point_idx);
}

std::pair<size_t, size_t> TargetBase::pointToGridCoordinates(
    size_t point_idx) const {
  return std::pair<size_t, size_t>(
      point_idx % cols(), static_cast<int>(point_idx / cols()));
}

size_t TargetBase::gridCoordinatesToPoint(
    size_t row_idx, size_t col_idx) const {
  CHECK_LT(row_idx, rows());
  CHECK_LT(col_idx, cols());
  const size_t point_idx = cols() * row_idx + col_idx;
  CHECK_LT(point_idx, size());
  return point_idx;
}

Eigen::Vector3d TargetBase::gridPoint(size_t row_idx, size_t col_idx) const {
  return points_target_frame_.col(gridCoordinatesToPoint(row_idx, col_idx));
}

double* TargetBase::pointMutable(size_t point_idx) {
  CHECK_LT(point_idx, size());
  return &points_target_frame_.coeffRef(0, point_idx);
}

double TargetBase::width() const {
  CHECK_GT(cols_, 0u);
  const Eigen::Vector3d vertical_difference =
      gridPoint(0u, cols_ - 1u) - gridPoint(0u, 0u);
  return std::abs(vertical_difference.x());
}

double TargetBase::height() const {
  CHECK_GT(rows_, 0u);
  const Eigen::Vector3d horizontal_difference =
      gridPoint(rows_ - 1u, 0u) - gridPoint(0u, 0u);
  return std::abs(horizontal_difference.y());
}

}  // namespace calibration
}  // namespace aslam
