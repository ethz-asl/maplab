#ifndef VIO_COMMON_VIO_TYPES_H_
#define VIO_COMMON_VIO_TYPES_H_

#include <string>
#include <utility>
#include <vector>

#include <Eigen/Core>
#include <aslam/common/memory.h>
#include <aslam/common/pose-types.h>
#include <aslam/common/time.h>
#include <aslam/frames/visual-nframe.h>
#include <localization-summary-map/localization-summary-map.h>
#include <maplab-common/interpolation-helpers.h>
#include <maplab-common/localization-result.h>
#include <maplab-common/macros.h>
#include <opencv2/core/core.hpp>

namespace vio {

enum class EstimatorState : int {
  kUninitialized,
  kStartup,
  kRunning,
  kInvalid
};
MAPLAB_DEFINE_ENUM_HASHING(EstimatorState, int);

enum class MotionType : int { kInvalid, kRotationOnly, kGeneralMotion };
MAPLAB_DEFINE_ENUM_HASHING(MotionType, int);

struct LocalizationResult : public common::LocalizationResult {
  MAPLAB_POINTER_TYPEDEFS(LocalizationResult);
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  LocalizationResult()
      : common::LocalizationResult(
            common::LocalizationType::kVisualFeatureBased) {}

  // Identifies the VisualNframe that was used to obtain this visual
  // localization.
  aslam::NFramesId nframe_id;

  // Identifies the localization map that was used for this localization.
  summary_map::LocalizationSummaryMapId summary_map_id;

  // 2D-3D constraints.
  Aligned<std::vector, Eigen::Matrix3Xd> G_landmarks_per_camera;
  Aligned<std::vector, Eigen::Matrix2Xd> keypoint_measurements_per_camera;

  bool isValid() const {
    if (G_landmarks_per_camera.empty() ||
        G_landmarks_per_camera.size() !=
            keypoint_measurements_per_camera.size()) {
      return false;
    }

    // Each keypoint measurement should have a valid landmark.
    for (size_t idx = 0u; idx < G_landmarks_per_camera.size(); ++idx) {
      if (G_landmarks_per_camera[idx].cols() !=
          keypoint_measurements_per_camera[idx].cols()) {
        return false;
      }
    }
    return true;
  }
};

struct ImageMeasurement {
  MAPLAB_POINTER_TYPEDEFS(ImageMeasurement);
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  int64_t timestamp;
  int camera_index;
  cv::Mat image;

  ImageMeasurement()
      : timestamp(aslam::time::getInvalidTime()), camera_index(-1) {}
};

// [accel, gyro] = [m/s^2, rad/s]
typedef Eigen::Matrix<double, 6, 1> ImuData;
struct ImuMeasurement {
  MAPLAB_POINTER_TYPEDEFS(ImuMeasurement);
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  ImuMeasurement() = default;
  ImuMeasurement(int64_t _timestamp, const ImuData& _imu_data)
      : timestamp(_timestamp), imu_data(_imu_data) {}
  int64_t timestamp;
  ImuData imu_data;
};

struct BatchedImuMeasurements {
  MAPLAB_POINTER_TYPEDEFS(BatchedImuMeasurements);
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  BatchedImuMeasurements() = default;
  Aligned<std::vector, ImuMeasurement> batch;
};

enum class UpdateType { kInvalid, kNormalUpdate, kZeroVelocityUpdate };

struct OdometryMeasurement {
  MAPLAB_POINTER_TYPEDEFS(OdometryMeasurement);
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  int64_t timestamp;
  // Velocities in odometry sensor frame.
  Eigen::Vector3d velocity_linear_O;
  Eigen::Vector3d velocity_angular_O;

  OdometryMeasurement() : timestamp(aslam::time::getInvalidTime()) {}
};

/// A data structure containing a VisualNFrame
struct SynchronizedNFrame {
 public:
  MAPLAB_POINTER_TYPEDEFS(SynchronizedNFrame);
  SynchronizedNFrame() {}
  SynchronizedNFrame(
      const aslam::VisualNFrame::Ptr& frame,
      const MotionType& motion_wrt_last_frame)
      : nframe(frame), motion_wrt_last_nframe(motion_wrt_last_frame) {}
  aslam::VisualNFrame::Ptr nframe;

  /// Additional information obtained during feature tracking.
  MotionType motion_wrt_last_nframe;
};

/// A data structure containing a VisualNFrame and synchronized IMU measurements
/// since this and the last processed nframe's timestamp. For integration the
/// first imu measurement is a copy of the last imu measurement of the last
/// SynchronizedNFrameImu message.
// TODO(schneith): Make this struct or at least the nframe thread safe.
struct SynchronizedNFrameImu {
 public:
  MAPLAB_POINTER_TYPEDEFS(SynchronizedNFrameImu);
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /// IMU measurements since the last nframe (including the last IMU measurement
  /// of the of the previous edge for integration).
  Eigen::Matrix<int64_t, 1, Eigen::Dynamic> imu_timestamps;
  Eigen::Matrix<double, 6, Eigen::Dynamic> imu_measurements;

  aslam::VisualNFrame::Ptr nframe;

  /// Additional information obtained during feature tracking.
  MotionType motion_wrt_last_nframe;
};

/// The state of a ViNode (pose, velocity and bias).
class ViNodeState {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  MAPLAB_POINTER_TYPEDEFS(ViNodeState);
  explicit ViNodeState(const aslam::Transformation& T_M_I)
      : timestamp_ns_(aslam::time::getInvalidTime()),
        T_M_I_(T_M_I),
        v_M_I_(Eigen::Vector3d::Zero()),
        acc_bias_(Eigen::Vector3d::Zero()),
        gyro_bias_(Eigen::Vector3d::Zero()),
        pose_covariance_(Eigen::Matrix<double, 6, 6>::Zero()),
        twist_covariance_(Eigen::Matrix<double, 6, 6>::Zero()) {
  }

  ViNodeState()
      : timestamp_ns_(aslam::time::getInvalidTime()),
        v_M_I_(Eigen::Vector3d::Zero()),
        acc_bias_(Eigen::Vector3d::Zero()),
        gyro_bias_(Eigen::Vector3d::Zero()),
        pose_covariance_(Eigen::Matrix<double, 6, 6>::Zero()),
        twist_covariance_(Eigen::Matrix<double, 6, 6>::Zero()) {
    T_M_I_.setIdentity();
  }

  ViNodeState(
      const aslam::Transformation& T_M_I, const Eigen::Vector3d& v_M_I,
      const Eigen::Vector3d& accelerometer_bias,
      const Eigen::Vector3d& gyro_bias)
      : timestamp_ns_(aslam::time::getInvalidTime()),
        T_M_I_(T_M_I),
        v_M_I_(v_M_I),
        acc_bias_(accelerometer_bias),
        gyro_bias_(gyro_bias),
        pose_covariance_(Eigen::Matrix<double, 6, 6>::Zero()),
        twist_covariance_(Eigen::Matrix<double, 6, 6>::Zero()) {
  }

  ViNodeState(
      int64_t timestamp_ns, const aslam::Transformation& T_M_I,
      const Eigen::Vector3d& v_M_I, const Eigen::Vector3d& accelerometer_bias,
      const Eigen::Vector3d& gyro_bias)
      : timestamp_ns_(timestamp_ns),
        T_M_I_(T_M_I),
        v_M_I_(v_M_I),
        acc_bias_(accelerometer_bias),
        gyro_bias_(gyro_bias),
        pose_covariance_(Eigen::Matrix<double, 6, 6>::Zero()),
        twist_covariance_(Eigen::Matrix<double, 6, 6>::Zero()) {
    CHECK(aslam::time::isValidTime(timestamp_ns));
  }

  ViNodeState(
      int64_t timestamp_ns, const aslam::Transformation& T_M_I,
      const Eigen::Vector3d& v_M_I, const Eigen::Vector3d& accelerometer_bias,
      const Eigen::Vector3d& gyro_bias,
      const Eigen::Matrix<double, 6, 6>& pose_covariance,
      const Eigen::Matrix<double, 6, 6>& twist_covariance)
      : timestamp_ns_(timestamp_ns),
        T_M_I_(T_M_I),
        v_M_I_(v_M_I),
        acc_bias_(accelerometer_bias),
        gyro_bias_(gyro_bias),
        pose_covariance_(pose_covariance),
        twist_covariance_(twist_covariance) {
    CHECK(aslam::time::isValidTime(timestamp_ns));
  }

  virtual ~ViNodeState() {}

  inline int64_t getTimestamp() const {
    return timestamp_ns_;
  }
  inline void setTimestamp(int64_t timestamp_ns) {
    CHECK_GE(timestamp_ns, 0);
    timestamp_ns_ = timestamp_ns;
  }

  inline const aslam::Transformation& get_T_M_I() const {
    return T_M_I_;
  }
  inline aslam::Transformation& get_T_M_I() {
    return T_M_I_;
  }
  inline const Eigen::Vector3d& get_v_M_I() const {
    return v_M_I_;
  }
  inline const Eigen::Vector3d& getAccBias() const {
    return acc_bias_;
  }
  inline const Eigen::Vector3d& getGyroBias() const {
    return gyro_bias_;
  }
  inline const Eigen::Matrix<double, 6, 6>& getPoseCovariance() const {
    return pose_covariance_;
  }
  inline const Eigen::Matrix<double, 6, 6>& getTwistCovariance() const {
    return twist_covariance_;
  }
  inline Eigen::Matrix<double, 6, 1> getImuBias() const {
    return (Eigen::Matrix<double, 6, 1>() << getAccBias(), getGyroBias())
        .finished();
  }

  inline void set_T_M_I(const aslam::Transformation& T_M_I) {
    T_M_I_ = T_M_I;
  }
  inline void set_v_M_I(const Eigen::Vector3d& v_M_I) {
    v_M_I_ = v_M_I;
  }
  inline void setAccBias(const Eigen::Vector3d& acc_bias) {
    acc_bias_ = acc_bias;
  }
  inline void setGyroBias(const Eigen::Vector3d& gyro_bias) {
    gyro_bias_ = gyro_bias;
  }
  inline void setPoseCovariance(
      const Eigen::Matrix<double, 6, 6>& pose_covariance) {
    pose_covariance_ = pose_covariance;
  }
  inline void setTwistCovariance(
      const Eigen::Matrix<double, 6, 6>& twist_covariance) {
    twist_covariance_ = twist_covariance;
  }

  /// The first 12x12 sub-matrix of the rovio state contains both
  /// the pose covariance matrix P and
  /// the twist covariance matrix T
  /// P3x3 0    P3x3
  /// 0    T6x6 0
  /// P3x3 0    P3x3
  inline void setCovariancesFromRovioMatrix(
      const Eigen::MatrixXd& rovio_imu_covariance) {
    CHECK_GE(rovio_imu_covariance.rows(), 12);
    CHECK_GE(rovio_imu_covariance.cols(), 12);
    // assemble the 6x6 pose covariance matrix from the sub-blocks
    pose_covariance_.block<3, 3>(0, 0) = rovio_imu_covariance.block<3, 3>(0, 0);
    pose_covariance_.block<3, 3>(0, 3) = rovio_imu_covariance.block<3, 3>(0, 9);
    pose_covariance_.block<3, 3>(3, 3) = rovio_imu_covariance.block<3, 3>(9, 9);
    pose_covariance_.block<3, 3>(3, 0) = rovio_imu_covariance.block<3, 3>(9, 0);
    twist_covariance_ = rovio_imu_covariance.block<6, 6>(3, 3);
  }

 private:
  int64_t timestamp_ns_;

  /// The pose taking points from the body frame to the world frame.
  aslam::Transformation T_M_I_;
  /// The velocity (m/s).
  Eigen::Vector3d v_M_I_;
  /// The accelerometer bias (m/s^2).
  Eigen::Vector3d acc_bias_;
  /// The gyroscope bias (rad/s).
  Eigen::Vector3d gyro_bias_;

  /// The 6DoF pose covariance matrix
  Eigen::Matrix<double, 6, 6> pose_covariance_;
  /// The 6DoF twist covariance matrix
  Eigen::Matrix<double, 6, 6> twist_covariance_;
};

typedef std::pair<aslam::NFramesId, ViNodeState> NFrameIdViNodeStatePair;
typedef Aligned<std::vector, NFrameIdViNodeStatePair> ViNodeStates;
typedef AlignedUnorderedMap<aslam::NFramesId, ViNodeState>
    NFrameIdViNodeStateMap;
typedef std::pair<aslam::VisualNFrame::Ptr, ViNodeState> NFrameViNodeStatePair;
}  // namespace vio

namespace common {
template <typename Time>
struct LinearInterpolationFunctor<Time, vio::ViNodeState> {
  void operator()(
      const Time t1, const vio::ViNodeState& x1, const Time t2,
      const vio::ViNodeState& x2, const Time t_interpolated,
      vio::ViNodeState* x_interpolated) {
    CHECK_NOTNULL(x_interpolated);

    x_interpolated->setTimestamp(t_interpolated);

    aslam::Transformation T_M_I_interpolated;
    interpolateTransformation(
        t1, x1.get_T_M_I(), t2, x2.get_T_M_I(), t_interpolated,
        &T_M_I_interpolated);
    x_interpolated->set_T_M_I(T_M_I_interpolated);

    Eigen::Vector3d v_M_I_interpolated;
    linearInterpolation(
        t1, x1.get_v_M_I(), t2, x2.get_v_M_I(), t_interpolated,
        &v_M_I_interpolated);
    x_interpolated->set_v_M_I(v_M_I_interpolated);

    Eigen::Vector3d acc_bias_interpolated;
    linearInterpolation(
        t1, x1.getAccBias(), t2, x2.getAccBias(), t_interpolated,
        &acc_bias_interpolated);
    x_interpolated->setAccBias(acc_bias_interpolated);

    Eigen::Vector3d gyro_bias_interpolated;
    linearInterpolation(
        t1, x1.getGyroBias(), t2, x2.getGyroBias(), t_interpolated,
        &gyro_bias_interpolated);
    x_interpolated->setGyroBias(gyro_bias_interpolated);
  }
};

template <typename Time>
struct LinearInterpolationFunctor<Time, Eigen::Vector3d> {
  void operator()(
      const Time t1, const Eigen::Vector3d& x1, const Time t2,
      const Eigen::Vector3d& x2, const Time t_interpolated,
      Eigen::Vector3d* x_interpolated) {
    CHECK_NOTNULL(x_interpolated);
    linearInterpolation(t1, x1, t2, x2, t_interpolated, x_interpolated);
  }
};
}  // namespace common

#include "./internal/vio-types-inl.h"
#endif  // VIO_COMMON_VIO_TYPES_H_
