#ifndef CERES_ERROR_TERMS_INERTIAL_ERROR_TERM_H_
#define CERES_ERROR_TERMS_INERTIAL_ERROR_TERM_H_

#include <vector>

#include <Eigen/Core>
#include <ceres/sized_cost_function.h>
#include <glog/logging.h>
#include <imu-integrator/imu-integrator.h>

#include <ceres-error-terms/common.h>

namespace ceres_error_terms {

typedef Eigen::Matrix<double, imu_integrator::kStateSize, 1>
    InertialStateVector;
typedef Eigen::Matrix<
    double, imu_integrator::kErrorStateSize, imu_integrator::kErrorStateSize>
    InertialStateCovariance;
typedef Eigen::Matrix<
    double, imu_integrator::kErrorStateSize, imu_integrator::kStateSize>
    InertialJacobianType;

struct InertialState {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Eigen::Matrix<double, 4, 1> q_I_M;
  Eigen::Matrix<double, 3, 1> b_g;
  Eigen::Matrix<double, 3, 1> v_M;
  Eigen::Matrix<double, 3, 1> b_a;
  Eigen::Matrix<double, 3, 1> p_M_I;

  bool operator==(const InertialState& other) const {
    return (q_I_M == other.q_I_M) && (b_g == other.b_g) && (v_M == other.v_M) &&
           (b_a == other.b_a) && (p_M_I == other.p_M_I);
  }

  InertialStateVector toVector() const {
    InertialStateVector vector;
    vector << q_I_M, b_g, v_M, b_a, p_M_I;
    return vector;
  }

  static InertialState fromVector(const InertialStateVector& vector) {
    InertialState state;
    state.q_I_M = vector.head<imu_integrator::kStateOrientationBlockSize>();
    state.b_g = vector.segment<imu_integrator::kGyroBiasBlockSize>(
        imu_integrator::kStateGyroBiasOffset);
    state.v_M = vector.segment<imu_integrator::kVelocityBlockSize>(
        imu_integrator::kStateVelocityOffset);
    state.b_a = vector.segment<imu_integrator::kAccelBiasBlockSize>(
        imu_integrator::kStateAccelBiasOffset);
    state.p_M_I = vector.segment<imu_integrator::kPositionBlockSize>(
        imu_integrator::kStatePositionOffset);
    return state;
  }
};

struct ImuIntegration {
  ImuIntegration() : valid(false) {}
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  InertialState begin_state;
  InertialState end_state;
  Eigen::Vector4d intrinsics;

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
class InertialErrorTerm
    : public ceres::SizedCostFunction<
          imu_integrator::kErrorStateSize, imu_integrator::kStatePoseBlockSize,
          imu_integrator::kGyroBiasBlockSize,
          imu_integrator::kVelocityBlockSize,
          imu_integrator::kAccelBiasBlockSize,
          imu_integrator::kStatePoseBlockSize,
          imu_integrator::kGyroBiasBlockSize,
          imu_integrator::kVelocityBlockSize,
          imu_integrator::kAccelBiasBlockSize,
          imu_integrator::kIntrinsicsBlockSize> {
 public:
  InertialErrorTerm(
      const Eigen::Matrix<double, 6, Eigen::Dynamic>& imu_data,
      const Eigen::Matrix<int64_t, 1, Eigen::Dynamic>& imu_timestamps,
      double gravity_magnitude)
      : imu_timestamps_(imu_timestamps),
        imu_data_(imu_data),
        integrator_(gravity_magnitude) {
    CHECK_GT(imu_data.cols(), 0);
    CHECK_EQ(imu_data.cols(), imu_timestamps.cols());
  }

  virtual ~InertialErrorTerm() {}

  virtual bool Evaluate(
      double const* const* parameters, double* residuals_ptr,
      double** jacobians) const;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 private:
  void IntegrateStateAndCovariance(
      const InertialState& current_state,
      const Eigen::Matrix<int64_t, 1, Eigen::Dynamic>& imu_timestamps,
      const Eigen::Matrix<double, 6, Eigen::Dynamic>& imu_data,
      InertialState* next_state, InertialStateCovariance* phi_accum,
      InertialStateCovariance* Q_accum) const;

  const Eigen::Matrix<int64_t, 1, Eigen::Dynamic> imu_timestamps_;
  const Eigen::Matrix<double, 6, Eigen::Dynamic> imu_data_;

  mutable imu_integrator::ImuIntegratorRK4 integrator_;

  // Cache the IMU integration to avoid unnecessary integrations.
  mutable ImuIntegration integration_cache_;
};

}  // namespace ceres_error_terms

#endif  // CERES_ERROR_TERMS_INERTIAL_ERROR_TERM_H_
