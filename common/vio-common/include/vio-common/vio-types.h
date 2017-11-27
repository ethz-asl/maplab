#ifndef VIO_COMMON_VIO_TYPES_H_
#define VIO_COMMON_VIO_TYPES_H_

#include <utility>
#include <vector>

#include <Eigen/Core>
#include <aslam/common/memory.h>
#include <aslam/common/pose-types.h>
#include <aslam/common/time.h>
#include <aslam/frames/visual-nframe.h>
#include <maplab-common/macros.h>
#include <opencv2/core/core.hpp>

namespace vio {

enum class EstimatorState : int { kUninitialized, kStartup, kRunning };
MAPLAB_DEFINE_ENUM_HASHING(EstimatorState, int);

enum class LocalizationState : int {
  // No reference map has been set, localization is not performed.
  kUninitialized,
  // Baseframe transformation has not yet been initialized.
  kNotLocalized,
  // Baseframe was initialized and global map matching is performed.
  kLocalized,
  // Map matching is performed using map tracking.
  kMapTracking
};
MAPLAB_DEFINE_ENUM_HASHING(LocalizationState, int);

enum class MotionType : int { kInvalid, kRotationOnly, kGeneralMotion };
MAPLAB_DEFINE_ENUM_HASHING(MotionType, int);

struct LocalizationResult {
  MAPLAB_POINTER_TYPEDEFS(LocalizationResult);
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  int64_t timestamp;
  aslam::NFramesId nframe_id;
  aslam::Transformation T_G_I_lc_pnp;

  enum class LocalizationMode { kGlobal, kMapTracking };
  LocalizationMode localization_type;
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

struct GpsLlhMeasurement {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  MAPLAB_POINTER_TYPEDEFS(GpsLlhMeasurement);

  GpsLlhMeasurement() = default;
  // Timestamp in nanoseconds.
  int64_t timestamp;
  // Position of the receiver in decimal degree and meter.
  Eigen::Vector3d gps_position_lat_lon_alt_deg_m;
  Eigen::Matrix3d gps_position_lat_lon_alt_covariance;
};

enum MeasurementType {
  kVision,
  kGps,
  kMagnetometer,
  kStatic_pressure,
  kDifferential_pressure
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
      : T_M_I_(T_M_I),
        v_M_I_(Eigen::Vector3d::Zero()),
        acc_bias_(Eigen::Vector3d::Zero()),
        gyro_bias_(Eigen::Vector3d::Zero()) {
    T_UTM_B_.setIdentity();
    T_UTM_I_.setIdentity();
  }

  ViNodeState()
      : v_M_I_(Eigen::Vector3d::Zero()),
        acc_bias_(Eigen::Vector3d::Zero()),
        gyro_bias_(Eigen::Vector3d::Zero()) {
    T_M_I_.setIdentity();
    T_UTM_B_.setIdentity();
    T_UTM_I_.setIdentity();
  }

  ViNodeState(const aslam::Transformation& T_M_I, const Eigen::Vector3d& v_M_I,
              const Eigen::Vector3d& accelerometer_bias,
              const Eigen::Vector3d& gyro_bias)
      : T_M_I_(T_M_I),
        v_M_I_(v_M_I),
        acc_bias_(accelerometer_bias),
        gyro_bias_(gyro_bias) {
    T_UTM_B_.setIdentity();
    T_UTM_I_.setIdentity();
  }

  virtual ~ViNodeState() {}

  inline const aslam::Transformation& get_T_M_I() const { return T_M_I_; }
  inline aslam::Transformation& get_T_M_I() { return T_M_I_; }
  inline const Eigen::Vector3d& get_v_M_I() const { return v_M_I_; }
  inline const Eigen::Vector3d& getAccBias() const {
    return acc_bias_;
  }
  inline const Eigen::Vector3d& getGyroBias() const {
    return gyro_bias_;
  }
  inline Eigen::Matrix<double, 6, 1> getImuBias() const {
    return (Eigen::Matrix<double, 6, 1>() << getAccBias(), getGyroBias())
        .finished();
  }
  inline const aslam::Transformation& get_T_UTM_I() const {
    return T_UTM_I_;
  }
  inline const aslam::Transformation& get_T_UTM_B() const {
    return T_UTM_B_;
  }

  inline void set_T_M_I(const aslam::Transformation& T_M_I) { T_M_I_ = T_M_I; }
  inline void set_v_M_I(const Eigen::Vector3d& v_M_I) { v_M_I_ = v_M_I; }
  inline void setAccBias(const Eigen::Vector3d& acc_bias) {
    acc_bias_ = acc_bias;
  }
  inline void setGyroBias(const Eigen::Vector3d& gyro_bias) {
    gyro_bias_ = gyro_bias;
  }
  inline void set_T_UTM_I(const aslam::Transformation& T_UTM_I) {
    T_UTM_I_ = T_UTM_I;
  }
  inline void set_T_UTM_B(const aslam::Transformation& T_UTM_B) {
    T_UTM_B_ = T_UTM_B;
  }

 private:
  /// The pose taking points from the body frame to the world frame.
  aslam::Transformation T_M_I_;
  /// The velocity (m/s).
  Eigen::Vector3d v_M_I_;
  /// The accelerometer bias (m/s^2).
  Eigen::Vector3d acc_bias_;
  /// The gyroscope bias (rad/s).
  Eigen::Vector3d gyro_bias_;

  /// Transformation of IMU wrt UTM reference frame.
  aslam::Transformation T_UTM_I_;
  /// Transformation of the body frame wrt the UTM reference frame.
  aslam::Transformation T_UTM_B_;
};

typedef std::pair<aslam::NFramesId, ViNodeState> NFrameIdViNodeStatePair;
typedef Aligned<std::vector, NFrameIdViNodeStatePair> ViNodeStates;
typedef AlignedUnorderedMap<aslam::NFramesId, ViNodeState>
    NFrameIdViNodeStateMap;
typedef std::pair<aslam::VisualNFrame::Ptr, ViNodeState> NFrameViNodeStatePair;

namespace constant {
static const double kUninitializedVariance = 1.0e12;
}

class ViNodeCovariance {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  MAPLAB_POINTER_TYPEDEFS(ViNodeCovariance);

  ViNodeCovariance(const Eigen::Matrix3d& p_M_I_covariance,
                   const Eigen::Matrix3d& q_M_I_covariance,
                   const Eigen::Matrix3d& v_M_I_covariance,
                   const Eigen::Matrix3d& I_acc_bias_covariance,
                   const Eigen::Matrix3d& I_gyro_bias_covariance)
      : p_M_I_covariance_(p_M_I_covariance),
        q_M_I_covariance_(q_M_I_covariance),
        v_M_I_covariance_(v_M_I_covariance),
        I_acc_bias_covariance_(I_acc_bias_covariance),
        I_gyro_bias_covariance_(I_gyro_bias_covariance) {}

  ViNodeCovariance()
      : p_M_I_covariance_(Eigen::Matrix3d::Identity() *
                          constant::kUninitializedVariance),
        q_M_I_covariance_(Eigen::Matrix3d::Identity() *
                          constant::kUninitializedVariance),
        v_M_I_covariance_(Eigen::Matrix3d::Identity() *
                          constant::kUninitializedVariance),
        I_acc_bias_covariance_(Eigen::Matrix3d::Identity() *
                               constant::kUninitializedVariance),
        I_gyro_bias_covariance_(Eigen::Matrix3d::Identity() *
                                constant::kUninitializedVariance) {}

  virtual ~ViNodeCovariance() {}

  inline const Eigen::Matrix3d& get_p_M_I_Covariance() const {
    return p_M_I_covariance_;
  }
  inline const Eigen::Matrix3d& get_q_M_I_Covariance() const {
    return q_M_I_covariance_;
  }
  inline const Eigen::Matrix3d& get_v_M_I_Covariance() const {
    return v_M_I_covariance_;
  }
  inline const Eigen::Matrix3d& getAccBiasCovariance() const {
    return I_acc_bias_covariance_;
  }
  inline const Eigen::Matrix3d& getGyroBiasCovariance() const {
    return I_gyro_bias_covariance_;
  }

  inline void set_p_M_I_Covariance(const Eigen::Matrix3d& p_M_I_covariance) {
    p_M_I_covariance_ = p_M_I_covariance;
  }
  inline void set_q_M_I_Covariance(const Eigen::Matrix3d& q_M_I_covariance) {
    q_M_I_covariance_ = q_M_I_covariance;
  }
  inline void set_v_M_I_Covariance(const Eigen::Matrix3d& v_M_I_covariance) {
    v_M_I_covariance_ = v_M_I_covariance;
  }
  inline void setAccBiasCovariance(
      const Eigen::Matrix3d& I_acc_bias_covariance) {
    I_acc_bias_covariance_ = I_acc_bias_covariance;
  }
  inline void setGyroBiasCovariance(
      const Eigen::Matrix3d& I_gyro_bias_covariance) {
    I_gyro_bias_covariance_ = I_gyro_bias_covariance;
  }

 private:
  /// Position covariance of body-frame origin expressed in the world frame. [m]
  Eigen::Matrix3d p_M_I_covariance_;
  /// Orientation covariance of body-frame wrt. the world frame expressed as
  /// variance of the
  /// corresponding Cayley-Klein parameters. [rad/s]
  Eigen::Matrix3d q_M_I_covariance_;
  /// The velocity of the body-frame origin expressed in the world frame. [m/s]
  Eigen::Matrix3d v_M_I_covariance_;
  /// The accelerometer bias covariance expressed in the IMU frame. [m/s^2]
  Eigen::Matrix3d I_acc_bias_covariance_;
  /// The gyroscope bias covariance expressed in the IMU frame. [rad/s]
  Eigen::Matrix3d I_gyro_bias_covariance_;
};

class ViNodeStateAndCovariance final : public ViNodeState,
                                       public ViNodeCovariance {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  MAPLAB_POINTER_TYPEDEFS(ViNodeStateAndCovariance);

  ViNodeStateAndCovariance() : ViNodeState(), ViNodeCovariance() {}

  ViNodeStateAndCovariance(const aslam::Transformation& T_M_I,
                           const Eigen::Vector3d& v_M_I,
                           const Eigen::Vector3d& accelerometer_bias,
                           const Eigen::Vector3d& gyro_bias,
                           const Eigen::Matrix3d& p_M_I_covariance,
                           const Eigen::Matrix3d& q_M_I_covariance,
                           const Eigen::Matrix3d& v_M_I_covariance,
                           const Eigen::Matrix3d& I_acc_bias_covariance,
                           const Eigen::Matrix3d& I_gyro_bias_covariance)
      : ViNodeState(T_M_I, v_M_I, accelerometer_bias, gyro_bias),
        ViNodeCovariance(p_M_I_covariance, q_M_I_covariance, v_M_I_covariance,
                         I_acc_bias_covariance, I_gyro_bias_covariance) {}

  ViNodeStateAndCovariance(
      const ViNodeState& state, const ViNodeCovariance& state_covariance)
      : ViNodeState(state), ViNodeCovariance(state_covariance) {}

  virtual ~ViNodeStateAndCovariance() {}
};
}  // namespace vio
#include "./internal/vio-types-inl.h"
#endif  // VIO_COMMON_VIO_TYPES_H_
