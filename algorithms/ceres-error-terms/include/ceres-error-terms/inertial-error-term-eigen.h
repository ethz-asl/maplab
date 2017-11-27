#ifndef CERES_ERROR_TERMS_INERTIAL_ERROR_TERM_EIGEN_H_
#define CERES_ERROR_TERMS_INERTIAL_ERROR_TERM_EIGEN_H_

#include <vector>

#include <Eigen/Core>
#include <ceres/sized_cost_function.h>
#include <glog/logging.h>
#include <imu-integrator/imu-integrator-eigen.h>

#include <ceres-error-terms/common.h>

namespace ceres_error_terms {

typedef Eigen::Matrix<double, imu_integrator::kStateSize, 1>
    InertialStateVector;
typedef Eigen::Matrix<double, imu_integrator::kErrorStateSize,
                      imu_integrator::kErrorStateSize>
    InertialStateCovariance;
typedef Eigen::Matrix<double, imu_integrator::kErrorStateSize,
                      imu_integrator::kStateSize>
    InertialJacobianType;

struct InertialStateEigen {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Eigen::Matrix<double, 4, 1> q_M_I;
  Eigen::Matrix<double, 3, 1> b_g;
  Eigen::Matrix<double, 3, 1> I_v_I;
  Eigen::Matrix<double, 3, 1> b_a;
  Eigen::Matrix<double, 3, 1> M_p_MI;

  bool operator==(const InertialStateEigen& other) const {
    return (q_M_I == other.q_M_I) && (b_g == other.b_g) &&
           (I_v_I == other.I_v_I) && (b_a == other.b_a) &&
           (M_p_MI == other.M_p_MI);
  }

  InertialStateVector toVector() const {
    InertialStateVector vector;
    vector.segment<imu_integrator::kStateOrientationBlockSize>(
        imu_integrator::kStateOrientationOffset) = q_M_I;
    vector.segment<imu_integrator::kGyroBiasBlockSize>(
        imu_integrator::kStateGyroBiasOffset) = b_g;
    vector.segment<imu_integrator::kVelocityBlockSize>(
        imu_integrator::kStateVelocityOffset) = I_v_I;
    vector.segment<imu_integrator::kAccelBiasBlockSize>(
        imu_integrator::kStateAccelBiasOffset) = b_a;
    vector.segment<imu_integrator::kPositionBlockSize>(
        imu_integrator::kStatePositionOffset) = M_p_MI;
    return vector;
  }

  static InertialStateEigen fromVector(const InertialStateVector& vector) {
    InertialStateEigen state;
    state.q_M_I = vector.segment<imu_integrator::kStateOrientationBlockSize>(
        imu_integrator::kStateOrientationOffset);
    state.b_g = vector.segment<imu_integrator::kGyroBiasBlockSize>(
        imu_integrator::kStateGyroBiasOffset);
    state.I_v_I = vector.segment<imu_integrator::kVelocityBlockSize>(
        imu_integrator::kStateVelocityOffset);
    state.b_a = vector.segment<imu_integrator::kAccelBiasBlockSize>(
        imu_integrator::kStateAccelBiasOffset);
    state.M_p_MI = vector.segment<imu_integrator::kPositionBlockSize>(
        imu_integrator::kStatePositionOffset);
    return state;
  }
};

struct ImuIntegrationEigen {
  ImuIntegrationEigen() : valid(false) {}
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  InertialStateEigen begin_state;
  InertialStateEigen end_state;

  InertialStateCovariance phi_accum;
  InertialStateCovariance Q_accum;
  Eigen::LLT<InertialStateCovariance> L_cholesky_Q_accum;

  InertialJacobianType J_end;
  InertialJacobianType J_begin;

  bool valid;
};

// Note: this error term accepts rotations expressed as quaternions
// in JPL convention [x, y, z, w]. This convention corresponds to the internal
// coefficient storage of Eigen so you can directly pass pointer to your
// Eigen quaternion data, e.g. your_eigen_quaternion.coeffs().data().
class InertialErrorTermEigen
    : public ceres::SizedCostFunction<
          imu_integrator::kErrorStateSize,
          imu_integrator::kStateOrientationBlockSize,
          imu_integrator::kPositionBlockSize,
          imu_integrator::kVelocityBlockSize, imu_integrator::kImuBiasBlockSize,
          imu_integrator::kStateOrientationBlockSize,
          imu_integrator::kPositionBlockSize,
          imu_integrator::kVelocityBlockSize,
          imu_integrator::kImuBiasBlockSize> {
 public:
  InertialErrorTermEigen(
      const Eigen::Matrix<double, 6, Eigen::Dynamic>& imu_data,
      const Eigen::Matrix<int64_t, 1, Eigen::Dynamic>& imu_timestamps_ns,
      double gyro_noise_sigma, double gyro_bias_sigma, double acc_noise_sigma,
      double acc_bias_sigma, double gravity_magnitude)
      : imu_timestamps_ns_(imu_timestamps_ns),
        imu_data_(imu_data),
        imu_covariance_cached_(nullptr),
        integrator_(
            gyro_noise_sigma, gyro_bias_sigma, acc_noise_sigma, acc_bias_sigma,
            gravity_magnitude) {
    CHECK_GT(imu_data.cols(), 0);
    CHECK_EQ(imu_data.cols(), imu_timestamps_ns.cols());

    CHECK_GT(gyro_noise_sigma, 0.0);
    CHECK_GT(gyro_bias_sigma, 0.0);
    CHECK_GT(acc_noise_sigma, 0.0);
    CHECK_GT(acc_bias_sigma, 0.0);
  }

  virtual ~InertialErrorTermEigen() {}

  virtual bool Evaluate(
      double const* const* parameters, double* residuals_ptr,
      double** jacobians) const;

  inline void setCachedImuCovariancePointer(
      Eigen::Matrix<double, 6, 6>* imu_covariance_cached) {
    imu_covariance_cached_ = imu_covariance_cached;
  }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 private:
  // Don't change the ordering of the enum elements, they have to be the
  // same as the order of the parameter blocks.
  enum {
    kIdxOrientationFrom,
    kIdxPositionFrom,
    kIdxVelocityFrom,
    kIdxImuBiasFrom,
    kIdxOrientationTo,
    kIdxPositionTo,
    kIdxVelocityTo,
    kIdxImuBiasTo
  };
  void IntegrateStateAndCovariance(
      const InertialStateEigen& current_state,
      const Eigen::Matrix<int64_t, 1, Eigen::Dynamic>& imu_timestamps,
      const Eigen::Matrix<double, 6, Eigen::Dynamic>& imu_data,
      InertialStateEigen* next_state, InertialStateCovariance* phi_accum,
      InertialStateCovariance* Q_accum) const;

  const Eigen::Matrix<int64_t, 1, Eigen::Dynamic> imu_timestamps_ns_;
  const Eigen::Matrix<double, 6, Eigen::Dynamic> imu_data_;
  Eigen::Matrix<double, 6, 6>* imu_covariance_cached_;

  const imu_integrator::ImuIntegratorEigen integrator_;

  // Cache the IMU integration to avoid unnecessary integrations.
  mutable ImuIntegrationEigen integration_cache_;
};

}  // namespace ceres_error_terms

#endif  // CERES_ERROR_TERMS_INERTIAL_ERROR_TERM_EIGEN_H_
