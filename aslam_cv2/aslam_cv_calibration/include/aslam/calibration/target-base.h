#ifndef ASLAM_CALIBRATION_TARGET_BASE_H
#define ASLAM_CALIBRATION_TARGET_BASE_H

#include <utility>
#include <vector>

#include <aslam/common/macros.h>
#include <Eigen/Core>
#include <opencv2/core/core.hpp>

namespace aslam {
namespace calibration {
class TargetObservation;

/// \class TargetBase
/// \brief Represents a calibration target with known geometry.
///
/// The class is a little limiting:
///  The target is supposed to be square such that each row has the same number of points.
///  Points along a row are supposed to be colinear.
///  Points along a column are supposed to be colinear.
class TargetBase {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  ASLAM_POINTER_TYPEDEFS(TargetBase);

 protected:
  TargetBase(size_t rows, size_t cols, const Eigen::Matrix3Xd& target_points);

 public:
  virtual ~TargetBase() {};

  /// Get all points from the target expressed in the target frame.
  const Eigen::Matrix3Xd& points() const;
  /// Get a point from the target expressed in the target frame.
  Eigen::Vector3d point(size_t point_idx) const;
  /// Return pointer to the i-th grid point in target frame.
  double* pointMutable(size_t point_idx);
  /// Get a point from the target expressed in the target frame by row and column.
  Eigen::Vector3d gridPoint(size_t row_idx, size_t col_idx) const;

  /// Get the grid coordinates for a point index.
  std::pair<size_t, size_t> pointToGridCoordinates(size_t point_idx) const;
  /// Get the point index from the grid coordinates.
  size_t gridCoordinatesToPoint(size_t row_idx, size_t col_idx) const;

  /// Number of rows in the calibration target.
  inline size_t rows() const { return rows_; };
  /// Number of columns in the calibration target.
  inline size_t cols() const { return cols_; };
  /// Get the number of points of the full grid.
  inline size_t size() const { return rows_ * cols_; };

  /// Extent of the grid in row dimension.
  virtual double width() const;
  /// Extent of the grid in column dimension.
  virtual double height() const;

 protected:
  /// Number of point rows in the calibration target.
  const size_t rows_;
  /// Number of point columns in the calibration target.
  const size_t cols_;

  /// Grid points stored in row-major order (idx = cols * r + c).
  Eigen::Matrix3Xd points_target_frame_;
}; //class TargetBase

class DetectorBase {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  ASLAM_POINTER_TYPEDEFS(DetectorBase);

 protected:
  DetectorBase() = default;

 public:
  virtual ~DetectorBase() {};

  /// Extract the calibration target points from an image.
  virtual std::shared_ptr<TargetObservation> detectTargetInImage(const cv::Mat& image) const = 0;
}; //class DetectorBase

}  // namespace calibration
}  // namespace aslam

#endif  // ASLAM_CALIBRATION_TARGET_BASE_H
