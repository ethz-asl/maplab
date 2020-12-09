#include "registration-toolbox/alignment/loam-alignment.h"

#include <utility>
#include <vector>

#include <map-resources/resource-conversion.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/filter.h>
#include <pcl_conversions/pcl_conversions.h>

DEFINE_int32(
    regbox_loam_optimization_iterations, 60,
    "Iterations for LOAM Optimization");
DEFINE_int32(
    regbox_loam_ceres_iterations, 10, "Iterations per Ceres Optimization");
DEFINE_double(
    regbox_loam_max_edge_distance_m, 1.0,
    "Maximum point distance for edge point matches");
DEFINE_double(
    regbox_loam_max_surface_distance_m, 1.0,
    "Maximum point distance for surface point matches");

namespace regbox {

RegistrationResult LoamAlignment::registerCloudImpl(
    const resources::PointCloud& target, const resources::PointCloud& source,
    const aslam::Transformation& prior_T_target_source) {
  pcl::PointCloud<pcl::PointXYZL>::Ptr source_pc(
      new pcl::PointCloud<pcl::PointXYZL>);
  pcl::PointCloud<pcl::PointXYZL>::Ptr target_pc(
      new pcl::PointCloud<pcl::PointXYZL>);
  backend::convertPointCloudType(source, source_pc.get());
  backend::convertPointCloudType(target, target_pc.get());

  extractFeaturesFromInputClouds(target_pc, source_pc);

  kd_tree_target_edges_ = pcl::KdTreeFLANN<pcl::PointXYZL>::Ptr(
      new pcl::KdTreeFLANN<pcl::PointXYZL>());
  kd_tree_target_surfaces_ = pcl::KdTreeFLANN<pcl::PointXYZL>::Ptr(
      new pcl::KdTreeFLANN<pcl::PointXYZL>());

  Eigen::Map<Eigen::Quaterniond> q_w_curr =
        Eigen::Map<Eigen::Quaterniond>(&parameters_[0]);
  Eigen::Map<Eigen::Vector3d> t_w_curr =
      Eigen::Map<Eigen::Vector3d>(&parameters_[0] + 4);

  q_w_curr = Eigen::Quaterniond(prior_T_target_source.getEigenQuaternion());
  t_w_curr = prior_T_target_source.getPosition();
  const Eigen::MatrixXd covariance = Eigen::MatrixXd::Identity(6, 6) * 1e-4;

  if ((target_edges_->empty() && target_surfaces_->empty()) ||
      (source_edges_->empty() && source_surfaces_->empty())) {
    return RegistrationResult(source, covariance, prior_T_target_source, false);
  }

  kd_tree_target_edges_->setInputCloud(target_edges_);
  kd_tree_target_surfaces_->setInputCloud(target_surfaces_);
  const size_t k_optimization_count = FLAGS_regbox_loam_optimization_iterations;
  for (int iterCount = 0; iterCount < k_optimization_count; iterCount++) {
    ceres::LossFunction* loss_function = new ceres::HuberLoss(0.2);
    ceres::Problem::Options problem_options;
    ceres::Problem problem(problem_options);
    problem.AddParameterBlock(parameters_, 7, new PoseSE3Parameterization());

    const aslam::Transformation estimated_T_target_source(q_w_curr, t_w_curr);

    addEdgeCostFactors(
        target_edges_, source_edges_, estimated_T_target_source, &problem,
        loss_function);
    addSurfaceCostFactors(
        target_surfaces_, source_surfaces_, estimated_T_target_source, &problem,
        loss_function);

    ceres::Solver::Options solver_options;
    solver_options.linear_solver_type = ceres::DENSE_QR;
    solver_options.minimizer_progress_to_stdout = false;
    solver_options.check_gradients = false;
    solver_options.gradient_check_relative_precision = 1e-4;
    ceres::Solver::Summary summary;

    ceres::Solve(solver_options, &problem, &summary);
  }
  const aslam::Transformation T_target_source(q_w_curr, t_w_curr);

  resources::PointCloud source_registered = source;
  source_registered.applyTransformation(T_target_source);

  return RegistrationResult(
      source_registered, covariance, T_target_source, true);
}

void LoamAlignment::extractFeaturesFromInputClouds(
    const PclPointCloudPtr<pcl::PointXYZL>& target,
    const PclPointCloudPtr<pcl::PointXYZL>& source) {
  target_surfaces_ =
      PclPointCloudPtr<pcl::PointXYZL>(new pcl::PointCloud<pcl::PointXYZL>);
  target_edges_ =
      PclPointCloudPtr<pcl::PointXYZL>(new pcl::PointCloud<pcl::PointXYZL>);
  source_surfaces_ =
      PclPointCloudPtr<pcl::PointXYZL>(new pcl::PointCloud<pcl::PointXYZL>);
  source_edges_ =
      PclPointCloudPtr<pcl::PointXYZL>(new pcl::PointCloud<pcl::PointXYZL>);

  for (pcl::PointXYZL point : target->points) {
    if (point.label == 0) {
      target_surfaces_->push_back(point);
    } else if (point.label == 1) {
      target_edges_->push_back(point);
    }
  }

  for (pcl::PointXYZL point : source->points) {
    if (point.label == 0) {
      source_surfaces_->push_back(point);
    } else if (point.label == 1) {
      source_edges_->push_back(point);
    }
  }
}

bool LoamAlignment::calculateSolutionCovariance(
    // This is still a draft as the covariance is calculated on the
    // quaternions, but covariance on the euler angles is needed
    ceres::Problem* problem, Eigen::Matrix<float, 7, 7>* covariance) {
  CHECK_NOTNULL(problem);
  CHECK_NOTNULL(covariance);

  ceres::Covariance::Options options;
  ceres::Covariance ceres_covariance(options);

  std::vector<std::pair<const double*, const double*>> covariance_pairs;
  covariance_pairs.push_back(
      std::make_pair(&parameters_[0] + 4, &parameters_[0] + 4));
  covariance_pairs.push_back(std::make_pair(&parameters_[0], &parameters_[0]));

  if (!ceres_covariance.Compute(covariance_pairs, problem)) {
    return false;
  }

  double covariance_q[16];
  ceres_covariance.GetCovarianceBlock(
      &parameters_[0] + 4, &parameters_[0] + 4, covariance_q);
  double covariance_t[9];
  ceres_covariance.GetCovarianceBlock(
      &parameters_[0], &parameters_[0], covariance_t);
}

bool EdgeAnalyticCostFunction::Evaluate(
    double const* const* parameters, double* residuals,
    double** jacobians) const {
  CHECK_NOTNULL(parameters);
  CHECK_NOTNULL(residuals);

  Eigen::Map<const Eigen::Quaterniond> q_target_source(parameters[0]);
  Eigen::Map<const Eigen::Vector3d> t_target_source(parameters[0] + 4);
  const Eigen::Vector3d point_on_line =
      q_target_source * curr_point_ + t_target_source;
  const Eigen::Vector3d cross_product =
      (point_on_line - last_point_a_).cross(point_on_line - last_point_b_);
  const Eigen::Vector3d line_direction = last_point_a_ - last_point_b_;

  residuals[0] = cross_product.x() / line_direction.norm();
  residuals[1] = cross_product.y() / line_direction.norm();
  residuals[2] = cross_product.z() / line_direction.norm();

  if (jacobians != NULL) {
    if (jacobians[0] != NULL) {
      const Eigen::Matrix3d point_on_line_skew = skew(point_on_line);
      Eigen::Matrix<double, 3, 6> dp_by_so3;
      dp_by_so3.block<3, 3>(0, 0) = -point_on_line_skew;
      (dp_by_so3.block<3, 3>(0, 3)).setIdentity();
      Eigen::Map<Eigen::Matrix<double, 3, 7, Eigen::RowMajor>> J_se3(
          jacobians[0]);
      J_se3.setZero();
      J_se3.block<3, 6>(0, 0) =
          skew(-line_direction) * dp_by_so3 / line_direction.norm();
    }
  }

  return true;
}

bool SurfaceAnalyticCostFunction::Evaluate(
    double const* const* parameters, double* residuals,
    double** jacobians) const {
  CHECK_NOTNULL(parameters);
  CHECK_NOTNULL(residuals);
  Eigen::Map<const Eigen::Quaterniond> q_w_curr(parameters[0]);
  Eigen::Map<const Eigen::Vector3d> t_w_curr(parameters[0] + 4);
  const Eigen::Vector3d point_w = q_w_curr * curr_point_ + t_w_curr;

  residuals[0] = plane_unit_norm_.dot(point_w) + negative_OA_dot_norm_;

  if (jacobians != NULL) {
    if (jacobians[0] != NULL) {
      Eigen::Matrix3d skew_point_w = skew(point_w);
      Eigen::Matrix<double, 3, 6> dp_by_so3;
      dp_by_so3.block<3, 3>(0, 0) = -skew_point_w;
      (dp_by_so3.block<3, 3>(0, 3)).setIdentity();
      Eigen::Map<Eigen::Matrix<double, 1, 7, Eigen::RowMajor>> J_se3(
          jacobians[0]);
      J_se3.setZero();
      J_se3.block<1, 6>(0, 0) = plane_unit_norm_.transpose() * dp_by_so3;
    }
  }
  return true;
}

bool PoseSE3Parameterization::Plus(
    const double* x, const double* delta, double* x_plus_delta) const {
  CHECK_NOTNULL(x);
  CHECK_NOTNULL(x_plus_delta);

  Eigen::Map<const Eigen::Vector3d> trans(x + 4);

  Eigen::Quaterniond delta_q;
  Eigen::Vector3d delta_t;
  getTransformFromSe3(
      Eigen::Map<const Eigen::Matrix<double, 6, 1>>(delta), &delta_q, &delta_t);
  Eigen::Map<const Eigen::Quaterniond> quater(x);
  Eigen::Map<Eigen::Quaterniond> quater_plus(x_plus_delta);
  Eigen::Map<Eigen::Vector3d> trans_plus(x_plus_delta + 4);

  quater_plus = delta_q * quater;
  trans_plus = delta_q * trans + delta_t;

  return true;
}

bool PoseSE3Parameterization::ComputeJacobian(
    const double* x, double* jacobian) const {
  CHECK_NOTNULL(x);
  CHECK_NOTNULL(jacobian);

  Eigen::Map<Eigen::Matrix<double, 7, 6, Eigen::RowMajor>> j(jacobian);
  (j.topRows(6)).setIdentity();
  (j.bottomRows(1)).setZero();

  return true;
}

void PoseSE3Parameterization::getTransformFromSe3(
    const Eigen::Matrix<double, 6, 1>& se3, Eigen::Quaterniond* q,
    Eigen::Vector3d* t) const {
  CHECK_NOTNULL(q);
  CHECK_NOTNULL(t);

  const Eigen::Vector3d omega(se3.data());
  const Eigen::Vector3d epsilon(se3.data() + 3);
  const Eigen::Matrix3d omega_skew = skew(omega);

  const double theta = omega.norm();

  double imag_factor;
  const double real_factor = cos(0.5 * theta);
  if (theta < 1e-10) {
    const double theta_squared = theta * theta;
    const double theta_squared_squared = theta_squared * theta_squared;
    // TODO(patripfr): find out where those numbers come from
    imag_factor =
        0.5 - 0.0208333 * theta_squared + 0.000260417 * theta_squared_squared;
  } else {
    const double sin_half_theta = sin(0.5 * theta);
    imag_factor = sin_half_theta / theta;
  }

  *q = Eigen::Quaterniond(real_factor, imag_factor*omega.x(),
    imag_factor*omega.y(), imag_factor*omega.z());


  Eigen::Matrix3d J;
  if (theta < 1e-10) {
    J = q->matrix();
  } else {
    const Eigen::Matrix3d omega_skew_squared = omega_skew * omega_skew;
    J =
        (Eigen::Matrix3d::Identity() +
         (1 - cos(theta)) / (theta * theta) * omega_skew +
         (theta - sin(theta)) / (pow(theta, 3)) * omega_skew_squared);
  }

  *t = J * epsilon;
}

void LoamAlignment::addEdgeCostFactors(
    const PclPointCloudPtr<pcl::PointXYZL>& target_edges,
    const PclPointCloudPtr<pcl::PointXYZL>& source_edges,
    const aslam::Transformation& T_target_source, ceres::Problem* problem,
    ceres::LossFunction* loss_function) {
  CHECK_NOTNULL(problem);
  CHECK_NOTNULL(loss_function);

  size_t n_corners = 0u;

  PclPointCloudPtr<pcl::PointXYZL> source_edges_transformed(
      new pcl::PointCloud<pcl::PointXYZL>);
  pcl::transformPointCloud(
      *source_edges, *source_edges_transformed,
      T_target_source.getTransformationMatrix());

  for (size_t source_point_idx = 0u;
       source_point_idx < source_edges->points.size(); source_point_idx++) {
    std::vector<int> point_search_indices;
    std::vector<float> point_search_distances_squared_m2;
    const size_t k_edge_points = 5u;
    kd_tree_target_edges_->nearestKSearch(
        source_edges_transformed->points[source_point_idx], k_edge_points,
        point_search_indices, point_search_distances_squared_m2);

    const float k_max_search_distance_squared_m2 =
        FLAGS_regbox_loam_max_edge_distance_m *
        FLAGS_regbox_loam_max_edge_distance_m;

    if (point_search_distances_squared_m2[k_edge_points - 1u] <
        k_max_search_distance_squared_m2) {
      std::vector<Eigen::Vector3d> near_corners;
      Eigen::Vector3d line_center(0, 0, 0);
      for (size_t search_result_idx = 0u; search_result_idx < k_edge_points;
           search_result_idx++) {
        const Eigen::Vector3d target_edge_point =
            target_edges->points[point_search_indices[search_result_idx]]
                .getVector3fMap()
                .cast<double>();
        line_center += target_edge_point;
        near_corners.push_back(target_edge_point);
      }
      line_center /= static_cast<float>(k_edge_points);

      Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
      for (size_t search_result_idx = 0u; search_result_idx < k_edge_points;
           search_result_idx++) {
        const Eigen::Vector3d p_corner_point_to_center =
            near_corners[search_result_idx] - line_center;
        covariance +=
            p_corner_point_to_center * p_corner_point_to_center.transpose();
      }

      Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> adjoint_solver(covariance);

      const Eigen::Vector3d unit_direction =
          adjoint_solver.eigenvectors().col(2);
      const Eigen::Vector3d edge_point = source_edges->points[source_point_idx]
                                             .getVector3fMap()
                                             .cast<double>();
      if (adjoint_solver.eigenvalues()[2] >
          3 * adjoint_solver.eigenvalues()[1]) {
        Eigen::Vector3d point_a, point_b;
        point_a = 0.1 * unit_direction + line_center;
        point_b = -0.1 * unit_direction + line_center;

        ceres::CostFunction* cost_function =
            new EdgeAnalyticCostFunction(edge_point, point_a, point_b);
        problem->AddResidualBlock(cost_function, loss_function, parameters_);
        n_corners++;
      }
    }
  }
  if (n_corners < 20) {
    // LOG(WARNING) << "Not enough edge points matched in Loam Alignment";
    // LOG(WARNING) << n_corners;
  }
}

void LoamAlignment::addSurfaceCostFactors(
    const PclPointCloudPtr<pcl::PointXYZL>& target_surfaces,
    const PclPointCloudPtr<pcl::PointXYZL>& source_surfaces,
    const aslam::Transformation& T_target_source, ceres::Problem* problem,
    ceres::LossFunction* loss_function) {
  CHECK_NOTNULL(problem);
  CHECK_NOTNULL(loss_function);

  size_t n_surfaces = 0;

  PclPointCloudPtr<pcl::PointXYZL> source_surfaces_transformed(
      new pcl::PointCloud<pcl::PointXYZL>);
  pcl::transformPointCloud(
      *source_surfaces, *source_surfaces_transformed,
      T_target_source.getTransformationMatrix());

  for (size_t source_point_idx = 0u;
       source_point_idx < source_surfaces->points.size(); source_point_idx++) {
    std::vector<int> point_search_indices;
    std::vector<float> point_search_distances_squared_m2;
    const size_t k_plane_points = 5u;
    kd_tree_target_surfaces_->nearestKSearch(
        source_surfaces_transformed->points[source_point_idx], k_plane_points,
        point_search_indices, point_search_distances_squared_m2);

    Eigen::Matrix<double, k_plane_points, 3> mat_A;
    Eigen::Matrix<double, k_plane_points, 1> mat_B =
        -Eigen::Matrix<double, k_plane_points, 1>::Ones();

    const float k_max_search_distance_squared_m2 =
        FLAGS_regbox_loam_max_surface_distance_m *
        FLAGS_regbox_loam_max_surface_distance_m;

    if (point_search_distances_squared_m2[k_plane_points - 1u] <
        k_max_search_distance_squared_m2) {
      for (size_t search_result_idx = 0u; search_result_idx < k_plane_points;
           search_result_idx++) {
        mat_A(search_result_idx, 0) =
            target_surfaces->points[point_search_indices[search_result_idx]].x;
        mat_A(search_result_idx, 1) =
            target_surfaces->points[point_search_indices[search_result_idx]].y;
        mat_A(search_result_idx, 2) =
            target_surfaces->points[point_search_indices[search_result_idx]].z;
      }

      // find the norm of the plane normal vector
      Eigen::Vector3d normal = mat_A.colPivHouseholderQr().solve(mat_B);
      const double negative_OA_dot_norm = 1 / normal.norm();
      normal.normalize();

      bool plane_valid = true;
      for (size_t search_result_idx = 0u; search_result_idx < k_plane_points;
           search_result_idx++) {
        const Eigen::Vector3d target_surface_point =
            target_surfaces->points[point_search_indices[search_result_idx]]
                .getVector3fMap()
                .cast<double>();
        const float distance_to_plane =
            fabs(normal.dot(target_surface_point) + negative_OA_dot_norm);
        const float k_max_distance_to_plane = 0.2;
        if (distance_to_plane > k_max_distance_to_plane) {
          plane_valid = false;
          break;
        }
      }

      Eigen::Vector3d source_point = source_surfaces->points[source_point_idx]
                                         .getVector3fMap()
                                         .cast<double>();
      if (plane_valid) {
        ceres::CostFunction* cost_function = new SurfaceAnalyticCostFunction(
            source_point, normal, negative_OA_dot_norm);
        problem->AddResidualBlock(cost_function, loss_function, parameters_);
        n_surfaces++;
      }
    }
  }
  if (n_surfaces < 20) {
    // LOG(WARNING) << "Not enough surface points matched in Loam Alignment";
  }
}

Eigen::Matrix<double, 3, 3> skew(const Eigen::Matrix<double, 3, 1>& mat_in) {
  Eigen::Matrix<double, 3, 3> skew_mat;
  skew_mat.setZero();
  skew_mat(0, 1) = -mat_in(2);
  skew_mat(0, 2) = mat_in(1);
  skew_mat(1, 2) = -mat_in(0);
  skew_mat(1, 0) = mat_in(2);
  skew_mat(2, 0) = -mat_in(1);
  skew_mat(2, 1) = mat_in(0);
  return skew_mat;
}

}  // namespace regbox
