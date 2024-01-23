#include "aslam/triangulation/triangulation.h"

#include <Eigen/QR>
#include <glog/logging.h>

namespace aslam {

TriangulationResult::Status TriangulationResult::SUCCESSFUL =
    TriangulationResult::Status::kSuccessful;
TriangulationResult::Status TriangulationResult::TOO_FEW_MEASUREMENTS =
    TriangulationResult::Status::kTooFewMeasurments;
TriangulationResult::Status TriangulationResult::UNOBSERVABLE =
    TriangulationResult::Status::kUnobservable;
TriangulationResult::Status TriangulationResult::UNINITIALIZED =
    TriangulationResult::Status::kUninitialized;

TriangulationResult linearTriangulateFromNViews(
    const Aligned<std::vector, Eigen::Vector2d>& measurements_normalized,
    const aslam::TransformationVector& T_G_B,
    const aslam::Transformation& T_B_C, Eigen::Vector3d* G_point) {
  CHECK_NOTNULL(G_point);
  CHECK_EQ(measurements_normalized.size(), T_G_B.size());
  if (measurements_normalized.size() < 2u) {
    return TriangulationResult(TriangulationResult::TOO_FEW_MEASUREMENTS);
  }

  VLOG(200) << "Triangulating from " << T_G_B.size() << " views.";

  const size_t rows = 3 * measurements_normalized.size();
  const size_t cols = 3 + measurements_normalized.size();
  Eigen::MatrixXd A = Eigen::MatrixXd::Zero(rows, cols);
  Eigen::VectorXd b = Eigen::VectorXd::Zero(rows);

  const Eigen::Matrix3d R_B_C = T_B_C.getRotationMatrix();

  // Fill in A and b.
  for (size_t i = 0; i < measurements_normalized.size(); ++i) {
    Eigen::Vector3d v(measurements_normalized[i](0),
        measurements_normalized[i](1), 1.);
    Eigen::Matrix3d R_G_B = T_G_B[i].getRotationMatrix();
    const Eigen::Vector3d& p_G_B = T_G_B[i].getPosition();
    A.block<3, 3>(3 * i, 0) = Eigen::Matrix3d::Identity();
    A.block<3, 1>(3 * i, 3 + i) = -R_G_B * R_B_C * v;
    b.segment<3>(3 * i) = p_G_B + R_G_B * T_B_C.getPosition();
  }

  Eigen::ColPivHouseholderQR<Eigen::MatrixXd> qr = A.colPivHouseholderQr();
  static constexpr double kRankLossTolerance = 0.001;
  qr.setThreshold(kRankLossTolerance);
  const size_t rank = qr.rank();

  if ((rank - measurements_normalized.size()) < 3) {
    return TriangulationResult(TriangulationResult::UNOBSERVABLE);
  }

  *G_point = qr.solve(b).head<3>();

  return TriangulationResult(TriangulationResult::SUCCESSFUL);
}

TriangulationResult linearTriangulateFromNViews(
    const Eigen::Matrix3Xd& t_G_bv,
    const Eigen::Matrix3Xd& p_G_C,
    Eigen::Vector3d* p_G_P) {
  CHECK_NOTNULL(p_G_P);

  const int num_measurements = t_G_bv.cols();
  if (num_measurements < 2) {
    return TriangulationResult(TriangulationResult::TOO_FEW_MEASUREMENTS);
  }

  // 1.) Formulate the geometrical problem
  // p_G_P + alpha[i] * t_G_bv[i] = p_G_C[i]      (+ alpha intended)
  // as linear system Ax = b, where
  // x = [p_G_P; alpha[0]; alpha[1]; ... ] and b = [p_G_C[0]; p_G_C[1]; ...]
  //
  // 2.) Apply the approximation AtAx = Atb
  // AtA happens to be composed of mostly more convenient blocks than A:
  // - Top left = N * Eigen::Matrix3d::Identity()
  // - Top right and bottom left = t_G_bv
  // - Bottom right = t_G_bv.colwise().squaredNorm().asDiagonal()

  // - Atb.head(3) = p_G_C.rowwise().sum()
  // - Atb.tail(N) = columnwise dot products between t_G_bv and p_G_C
  //               = t_G_bv.cwiseProduct(p_G_C).colwise().sum().transpose()
  //
  // 3.) Apply the Schur complement to solve after p_G_P only
  // AtA = [E B; C D] (same blocks as above) ->
  // (E - B * D.inverse() * C) * p_G_P = Atb.head(3) - B * D.inverse() * Atb.tail(N)

  const Eigen::MatrixXd BiD = t_G_bv *
      t_G_bv.colwise().squaredNorm().asDiagonal().inverse();
  const Eigen::Matrix3d AxtAx = num_measurements * Eigen::Matrix3d::Identity() -
      BiD * t_G_bv.transpose();
  const Eigen::Vector3d Axtbx = p_G_C.rowwise().sum() - BiD *
      t_G_bv.cwiseProduct(p_G_C).colwise().sum().transpose();

  Eigen::ColPivHouseholderQR<Eigen::Matrix3d> qr = AxtAx.colPivHouseholderQr();
  static constexpr double kRankLossTolerance = 1e-5;
  qr.setThreshold(kRankLossTolerance);
  const size_t rank = qr.rank();
  if (rank < 3) {
    return TriangulationResult(TriangulationResult::UNOBSERVABLE);
  }

  *p_G_P = qr.solve(Axtbx);
  return TriangulationResult(TriangulationResult::SUCCESSFUL);
}

TriangulationResult linearTriangulateFromNViewsMultiCam(
    const Aligned<std::vector, Eigen::Vector2d>& measurements_normalized,
    const std::vector<size_t>& measurement_camera_indices,
    const Aligned<std::vector, aslam::Transformation>& T_G_B,
    const Aligned<std::vector, aslam::Transformation>& T_B_C,
    Eigen::Vector3d* G_point) {
  CHECK_NOTNULL(G_point);
  CHECK_EQ(measurements_normalized.size(), T_G_B.size());
  CHECK_EQ(measurements_normalized.size(), measurement_camera_indices.size());
  if (measurements_normalized.size() < 2u) {
    return TriangulationResult(TriangulationResult::TOO_FEW_MEASUREMENTS);
  }

  const size_t rows = 3 * measurements_normalized.size();
  const size_t cols = 3 + measurements_normalized.size();
  Eigen::MatrixXd A = Eigen::MatrixXd::Zero(rows, cols);
  Eigen::VectorXd b = Eigen::VectorXd::Zero(rows);

  // Fill in A and b.
  for (size_t i = 0; i < measurements_normalized.size(); ++i) {
    size_t cam_index = measurement_camera_indices[i];
    CHECK_LT(cam_index, T_B_C.size());
    Eigen::Vector3d v(measurements_normalized[i](0),
        measurements_normalized[i](1), 1.);
    const Eigen::Vector3d& t_B_C = T_B_C[cam_index].getPosition();

    A.block<3, 3>(3 * i, 0) = Eigen::Matrix3d::Identity();
    A.block<3, 1>(3 * i, 3 + i) = -1.0 * T_G_B[i].getRotation().rotate(
        T_B_C[cam_index].getRotation().rotate(v));
    b.segment<3>(3 * i) = T_G_B[i] * t_B_C;
  }

  Eigen::ColPivHouseholderQR<Eigen::MatrixXd> qr = A.colPivHouseholderQr();
  static constexpr double kRankLossTolerance = 0.001;
  qr.setThreshold(kRankLossTolerance);
  const size_t rank = qr.rank();
  if ((rank - measurements_normalized.size()) < 3) {
    return TriangulationResult(TriangulationResult::UNOBSERVABLE);
  }

  *G_point = qr.solve(b).head<3>();
  return TriangulationResult(TriangulationResult::SUCCESSFUL);
}

// [1]  A. I. Mourikis and S. I. Roumeliotis, “A multi-state constraint kalman filter
// "for vision-aided inertial navigation,” in Proc. IEEE Int. Conf. on Robotics
// and Automation, pp. 10–14, 2007.
// [2] T. Hinzmann, "Robust Vision-Based Navigation for Micro Air Vehicles", 2014.
TriangulationResult iterativeGaussNewtonTriangulateFromNViews(
    const Aligned<std::vector, Eigen::Vector2d>& measurements_normalized,
    const Aligned<std::vector, aslam::Transformation>& T_G_B,
    const aslam::Transformation& T_B_C, Eigen::Vector3d* G_point) {
  CHECK_NOTNULL(G_point);
  CHECK_EQ(measurements_normalized.size(), T_G_B.size());
  if (measurements_normalized.size() < 2u) {
    return TriangulationResult(TriangulationResult::TOO_FEW_MEASUREMENTS);
  }

  const double kPrecision = 1.0e-5;
  const size_t kIterMax = 10;

  // Initialize minimization variables.
  double alpha = 0.0;
  double beta = 0.0;
  double rho = 0.0;

  double residual_norm_last = 1000.0;
  double residual_norm = 100.0;

  // Camera frame n: camera frame in which feature was observed for the first time.
  const size_t n = 0;

  // Rotation and position of n-th camera in i-th camera frame.
  const aslam::Transformation T_Cn_G = T_B_C.inverse() * T_G_B[n].inverse();
  const Eigen::Matrix3d& R_Cn_G = T_Cn_G.getRotationMatrix();
  const Eigen::Vector3d& p_G_Cn = T_Cn_G.getPosition();

  // Cache matrices that are constant for every iteration.
  Aligned<std::vector, Eigen::Matrix3d> R_Ci_Cn;
  Aligned<std::vector, Eigen::Vector3d> p_Ci_Cn;
  for (size_t i = 0; i < measurements_normalized.size(); ++i) {
    const aslam::Transformation T_Ci_G = T_B_C.inverse() * T_G_B[i].inverse();
    // Rotation from first camera to current camera.
    const Eigen::Matrix3d R_Ci_G = T_Ci_G.getRotationMatrix();
    R_Ci_Cn.emplace_back(R_Ci_G * R_Cn_G.transpose());
    // Translation from first camera to current camera.
    const Eigen::Vector3d p_G_Ci = T_Ci_G.inverse().getPosition();
    p_Ci_Cn.emplace_back(R_Ci_G * p_G_Cn - R_Ci_G * p_G_Ci);
  }

  // [1.] Loop over iterations.
  // Loop while delta residual too large or number of maximum iterations reached.
  size_t iter = 0;
  while (residual_norm_last - residual_norm > kPrecision && iter < kIterMax) {
    const size_t num_measurements = 2 * measurements_normalized.size();
    Eigen::VectorXd residuals(num_measurements);
    Eigen::MatrixXd jacobian(num_measurements, 3);
    residuals.setZero(num_measurements);
    jacobian.setZero(num_measurements, 3);

    // [2.] Loop over camera frames / measurements.
    for (size_t i = 0; i < measurements_normalized.size(); ++i) {
      // Current measurement.
      const Eigen::Vector2d& h_meas = measurements_normalized[i];

      // Predicted measurement.
      const Eigen::Vector3d h_i = R_Ci_Cn[i] *
	  (Eigen::Matrix<double, 3, 1>() << alpha, beta, 1.0).finished() + rho * p_Ci_Cn[i];
      // Normalized predicted measurement.
      const Eigen::Vector2d h  = h_i.head<2>() / h_i(2);

      // Calculate residuals.
      residuals.segment<2>(i * 2) = h_meas - h;

      // Calculate jacobians.
      Eigen::Matrix<double, 2, 3> jacobian_perspective;
      jacobian_perspective << -1.0 / h_i(2), 0.0, h_i(0) / (h_i(2) * h_i(2)),
          0.0, -1.0 / h_i(2), h_i(1) / (h_i(2) * h_i(2));

      const Eigen::Matrix<double, 3, 1> jacobian_alpha =
          R_Ci_Cn[i] * (Eigen::Matrix<double, 3, 1>() << 1.0, 0.0, 0.0).finished();
      const Eigen::Matrix<double, 3, 1> jacobian_beta =
          R_Ci_Cn[i] * (Eigen::Matrix<double, 3, 1>() << 0.0, 1.0, 0.0).finished();
      const Eigen::Matrix<double, 3, 1> jacobian_rho = p_Ci_Cn[i];

      const Eigen::Matrix<double, 2, 1> jacobian_A = jacobian_perspective * jacobian_alpha;
      const Eigen::Matrix<double, 2, 1> jacobian_B = jacobian_perspective * jacobian_beta;
      const Eigen::Matrix<double, 2, 1> jacobian_C = jacobian_perspective * jacobian_rho;

      jacobian.block<1, 3>(i * 2, 0) =
          (Eigen::Matrix<double, 1, 3>() << jacobian_A(0), jacobian_B(0), jacobian_C(0)).finished();
      jacobian.block<1, 3>(i * 2 + 1, 0) =
          (Eigen::Matrix<double, 1, 3>() << jacobian_A(1), jacobian_B(1), jacobian_C(1)).finished();
    }  // Measurement loop.

    // Calculate update using LDLT decomposition.
    Eigen::Vector3d delta = (jacobian.transpose() * jacobian)
	.ldlt().solve(jacobian.transpose() * residuals);

    alpha = alpha - delta(0);
    beta = beta - delta(1);
    rho = rho - delta(2);

    residual_norm_last = residual_norm;
    residual_norm = residuals.squaredNorm();
    ++iter;
  } // Iteration loop.

  // Coordinate of feature in global frame.
  *G_point = 1.0 / rho * R_Cn_G.transpose() *
      (Eigen::Matrix<double, 3, 1>() << alpha, beta, 1.0).finished() + p_G_Cn;
  return TriangulationResult(TriangulationResult::SUCCESSFUL);
}

TriangulationResult triangulateFeatureTrack(
    const aslam::FeatureTrack& track,
    const aslam::TransformationVector& T_W_Bs,
    Eigen::Vector3d* W_landmark) {
  CHECK_NOTNULL(W_landmark);
  size_t track_length = track.getTrackLength();
  CHECK_GT(track_length, 1u);
  CHECK_EQ(track_length, T_W_Bs.size());

  VLOG(200) << "Triangulating track of length " << track_length;

  const aslam::Camera::ConstPtr& camera = track.getFirstKeypointIdentifier().getCamera();
  CHECK(camera);
  aslam::Transformation T_B_C = track.getFirstKeypointIdentifier().get_T_C_B().inverse();

  // Get the normalized measurements for all observations on the track.
  Aligned<std::vector, Eigen::Vector2d> normalized_measurements;
  normalized_measurements.reserve(track_length);

  for (const aslam::KeypointIdentifier& keypoint_on_track : track.getKeypointIdentifiers()) {
    const aslam::Camera::ConstPtr& camera = keypoint_on_track.getCamera();
    CHECK(camera) << "Missing camera for keypoint on track with frame index: "
        << keypoint_on_track.getFrameIndex();

    // Obtain the normalized keypoint measurements.
    const Eigen::Vector2d& keypoint_measurement = keypoint_on_track.getKeypointMeasurement();
    Eigen::Vector3d C_ray;
    camera->backProject3(keypoint_measurement, &C_ray);
    Eigen::Vector2d normalized_measurement = C_ray.head<2>() / C_ray[2];
    normalized_measurements.push_back(normalized_measurement);
  }

  VLOG(200) << "Assembled triangulation data.";

  // Triangulate the landmark.
  CHECK_EQ(track_length, normalized_measurements.size());
  CHECK_EQ(track_length, T_W_Bs.size());
  aslam::TriangulationResult triangulation_result = linearTriangulateFromNViews(
                                                        normalized_measurements,
                                                        T_W_Bs,
                                                        T_B_C,
                                                        W_landmark);

  VLOG(200) << "Triangulation returned the following result:" << std::endl
          << triangulation_result;

  return triangulation_result;
}

TriangulationResult fastTriangulateFeatureTrack(
    const aslam::FeatureTrack& track,
    const aslam::TransformationVector& T_W_Bs,
    Eigen::Vector3d* W_landmark) {
  CHECK_NOTNULL(W_landmark);
  size_t track_length = track.getTrackLength();
  CHECK_GT(track_length, 1u);
  CHECK_EQ(track_length, T_W_Bs.size());

  VLOG(200) << "Triangulating track of length " << track_length;

  const aslam::Camera::ConstPtr& camera = track.getFirstKeypointIdentifier().getCamera();
  CHECK(camera);
  aslam::Transformation T_B_C = track.getFirstKeypointIdentifier().get_T_C_B().inverse();

  Eigen::Matrix3Xd G_bearing_vectors;
  Eigen::Matrix3Xd p_G_C_vector;

  G_bearing_vectors.resize(Eigen::NoChange, track_length);
  p_G_C_vector.resize(Eigen::NoChange, track_length);

  size_t index = 0u;
  for (const aslam::KeypointIdentifier& keypoint_on_track : track.getKeypointIdentifiers()) {
    const aslam::Camera::ConstPtr& camera = keypoint_on_track.getCamera();
    CHECK(camera) << "Missing camera for keypoint on track with frame index: "
        << keypoint_on_track.getFrameIndex();

    // Obtain the normalized keypoint measurements.
    const Eigen::Vector2d& keypoint_measurement = keypoint_on_track.getKeypointMeasurement();
    Eigen::Vector3d C_ray;
    camera->backProject3(keypoint_measurement, &C_ray);

    aslam::Transformation T_W_C = T_W_Bs[index] * keypoint_on_track.get_T_C_B().inverse();

    G_bearing_vectors.col(index) = T_W_C.getRotationMatrix() * C_ray;
    p_G_C_vector.col(index) = T_W_C.getPosition();
  }

  VLOG(200) << "Assembled triangulation data.";

  // Triangulate the landmark.
  aslam::TriangulationResult triangulation_result = linearTriangulateFromNViews(
      G_bearing_vectors, p_G_C_vector, W_landmark);

  VLOG(200) << "Triangulation returned the following result:" << std::endl
          << triangulation_result;

  return triangulation_result;
}

}  // namespace aslam
