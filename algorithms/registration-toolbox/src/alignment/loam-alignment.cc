#include "registration-toolbox/alignment/loam-alignment.h"

#include <pcl/common/transforms.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>

DEFINE_int32(
    regbox_loam_optimization_iterations, 30,
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
    const PclPointCloudPtr<pcl::PointXYZI>& target,
    const PclPointCloudPtr<pcl::PointXYZI>& source,
    const aslam::Transformation& prior_T_target_source) {

  extractFeaturesFromInputClouds(target, source);

  kd_tree_target_edges_ = pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr(
    new pcl::KdTreeFLANN<pcl::PointXYZI>());
  kd_tree_target_surfaces_ = pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr(
    new pcl::KdTreeFLANN<pcl::PointXYZI>());

  Eigen::Map<Eigen::Quaterniond> q_w_curr =
        Eigen::Map<Eigen::Quaterniond>(&parameters_[0]);
  Eigen::Map<Eigen::Vector3d> t_w_curr =
      Eigen::Map<Eigen::Vector3d>(&parameters_[0] + 4);

  q_w_curr = Eigen::Quaterniond(prior_T_target_source.getEigenQuaternion());
  t_w_curr = prior_T_target_source.getPosition();

  if (!(target_edges_->empty() && target_surfaces_->empty())) {
    std::cout << "edges: " << target_edges_->size() << std::endl;
    std::cout << "surfaces: " << target_surfaces_->size() << std::endl;
    kd_tree_target_edges_->setInputCloud(target_edges_);
    kd_tree_target_surfaces_->setInputCloud(target_surfaces_);

    const size_t k_optimization_count =
        FLAGS_regbox_loam_optimization_iterations;
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
          target_surfaces_, source_surfaces_, estimated_T_target_source,
          &problem, loss_function);

      ceres::Solver::Options solver_options;
      solver_options.linear_solver_type = ceres::DENSE_QR;
      // solver_options.max_num_iterations = FLAGS_regbox_loam_ceres_iterations;
      solver_options.minimizer_progress_to_stdout = false;
      solver_options.check_gradients = false;
      solver_options.gradient_check_relative_precision = 1e-4;
      ceres::Solver::Summary summary;

      ceres::Solve(solver_options, &problem, &summary);
    }
  }

  const aslam::Transformation T_target_source(q_w_curr, t_w_curr);

  pcl::PointCloud<pcl::PointXYZI> source_features;
  for (pcl::PointXYZI point : *source_surfaces_) {
    point.intensity = 0;
    source_features.push_back(point);
  }
  for (pcl::PointXYZI point : *source_edges_) {
    point.intensity = 1;
    source_features.push_back(point);
  }

  PclPointCloudPtr<pcl::PointXYZI> source_registered(
      new pcl::PointCloud<pcl::PointXYZI>);
  pcl::transformPointCloud(
      source_features, *source_registered,
      T_target_source.getTransformationMatrix());
  const Eigen::MatrixXd covariance = Eigen::MatrixXd::Identity(6, 6) * 1e-4;

  return RegistrationResult(
      source_registered, covariance, T_target_source, true);
}

void LoamAlignment::extractFeaturesFromInputClouds(
    const PclPointCloudPtr<pcl::PointXYZI>& target,
    const PclPointCloudPtr<pcl::PointXYZI>& source) {

  target_surfaces_ = PclPointCloudPtr<pcl::PointXYZI>(
      new pcl::PointCloud<pcl::PointXYZI>);
  target_edges_ = PclPointCloudPtr<pcl::PointXYZI>(
      new pcl::PointCloud<pcl::PointXYZI>);

  for (pcl::PointXYZI point : target->points) {
    if (point.intensity == 0) {
      target_surfaces_->push_back(point);
    } else if (point.intensity == 1) {
      target_edges_->push_back(point);
    }
  }

  source_surfaces_ = PclPointCloudPtr<pcl::PointXYZI>(
      new pcl::PointCloud<pcl::PointXYZI>);
  source_edges_ = PclPointCloudPtr<pcl::PointXYZI>(
      new pcl::PointCloud<pcl::PointXYZI>);

  extractFeaturesFromSourceCloud(source, source_edges_, source_surfaces_);
  //
  pcl::PointCloud<pcl::PointXYZI> source_edges_down_sampled;
  pcl::VoxelGrid<pcl::PointXYZI> edge_filter;
  edge_filter.setInputCloud(source_edges_);
  edge_filter.setLeafSize(0.2, 0.2, 0.2);
  edge_filter.filter(source_edges_down_sampled);
  *source_edges_ = source_edges_down_sampled;

  pcl::PointCloud<pcl::PointXYZI> source_surfaces_down_sampled;
  pcl::VoxelGrid<pcl::PointXYZI> surface_filter;
  surface_filter.setInputCloud(source_surfaces_);
  surface_filter.setLeafSize(0.4, 0.4, 0.4);
  surface_filter.filter(source_surfaces_down_sampled);
  *source_surfaces_ = source_surfaces_down_sampled;
  // for (pcl::PointXYZI point : source->points) {
  //   if (point.intensity == 0) {
  //     source_surfaces_->push_back(point);
  //   } else if (point.intensity == 1) {
  //     source_edges_->push_back(point);
  //   }
  // }
}

void LoamAlignment::extractFeaturesFromSourceCloud(
    const PclPointCloudPtr<pcl::PointXYZI>& source,
    PclPointCloudPtr<pcl::PointXYZI> source_edges,
    PclPointCloudPtr<pcl::PointXYZI> source_surfaces) {
  std::vector<int> indices;

  int N_SCANS = 128;
  double upperBound = 46.2f;
  double lowerBound = -46.2f;
  double factor = (N_SCANS - 1) / (upperBound - lowerBound);
  std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> scan_lines;

  for (int idx = 0u; idx < N_SCANS; idx++) {
    scan_lines.push_back(pcl::PointCloud<pcl::PointXYZI>::Ptr(
        new pcl::PointCloud<pcl::PointXYZI>()));
  }

  int min_id = 1000;
  int max_id = 0;
  for (int idx = 0u; idx < source->size(); idx++) {
    int scanID = 0;
    const double distance = sqrt(
        source->points[idx].x * source->points[idx].x +
        source->points[idx].y * source->points[idx].y);
    if (distance < 0.001 || distance > 120.)
      continue;
    if (!std::isfinite(source->points[idx].x) ||
        !std::isfinite(source->points[idx].y) ||
        !std::isfinite(source->points[idx].z)) {
      continue;
    }
    double angle = atan(source->points[idx].z / distance) * 180. / M_PI;

    if (N_SCANS == 128) {
      // // scanID = int(((angle * 180 / M_PI) - lowerBound) * factor + 0.5);
      // double distance = col_dist.norm();
      // double angle = atan(col_dist.z() / distance);
      // scanID = int(((angle) - lowerBound) * factor + 0.5);
      scanID = source->points[idx].intensity;
      min_id = std::min(scanID, min_id);
      max_id = std::max(scanID, max_id);
    }
    scan_lines[scanID]->push_back(source->points[idx]);
  }
  const int curvature_region = 5;
  const int feature_regions = 6;
  for (int line_idx = 0u; line_idx < N_SCANS; line_idx++) {
    if (scan_lines[line_idx]->points.size() < 131) {
      continue;
    }

    std::vector<bool> point_picked(scan_lines[line_idx]->points.size(), false);

    CurvaturePairs cloud_curvatures;
    int total_points =
        scan_lines[line_idx]->points.size() - 2 * curvature_region;
    for (int point_idx = curvature_region;
         point_idx < scan_lines[line_idx]->points.size() - curvature_region;
         point_idx++) {
      if (scan_lines[line_idx]->points[point_idx].getVector3fMap().norm() <
          0.8) {
        point_picked[point_idx] = true;
        for (int k = 1; k <= 5; k++) {
          point_picked[point_idx + k] = true;
          point_picked[point_idx - k] = true;
        }
      }

      Eigen::Vector3f merged_point = Eigen::Vector3f::Zero();
      for (int neighbor_idx = -curvature_region;
           neighbor_idx <= curvature_region; neighbor_idx++) {
        if (neighbor_idx == 0) {
          merged_point -= 2. * curvature_region *
                          scan_lines[line_idx]
                              ->points[point_idx + neighbor_idx]
                              .getVector3fMap();
        } else {
          merged_point += scan_lines[line_idx]
                              ->points[point_idx + neighbor_idx]
                              .getVector3fMap();
        }
      }

      CurvaturePair curvature_pair(point_idx, merged_point.norm());
      cloud_curvatures.push_back(curvature_pair);
    }
    int sector_length = total_points / feature_regions;

    for (int region_idx = 0; region_idx < feature_regions; region_idx++) {
      int sector_start = sector_length * region_idx;
      int sector_end = sector_length * (region_idx + 1) - 1;

      for (size_t i = sector_start + curvature_region;
           i < sector_end - curvature_region; i++) {
        const pcl::PointXYZI& previousPoint =
            (scan_lines[line_idx]->points[i - 1]);
        const pcl::PointXYZI& point = (scan_lines[line_idx]->points[i]);
        const pcl::PointXYZI& nextPoint = (scan_lines[line_idx]->points[i + 1]);

        float diffNext =
            (nextPoint.getVector3fMap() - point.getVector3fMap()).squaredNorm();

        if (diffNext > 0.1) {
          float depth1 = point.getVector3fMap().norm();
          float depth2 = nextPoint.getVector3fMap().norm();

          if (depth1 > depth2) {
            float weighted_distance =
                (nextPoint.getVector3fMap() -
                 (depth2 / depth1) * point.getVector3fMap())
                    .norm() /
                depth2;

            if (weighted_distance < 0.1) {
              for (int j = 0; j < curvature_region; j++) {
                point_picked[i - curvature_region + j] = true;
              }
              continue;
            }
          } else {
            float weighted_distance =
                (point.getVector3fMap() -
                 (depth1 / depth2) * nextPoint.getVector3fMap())
                    .norm() /
                depth1;
            if (weighted_distance < 0.1) {
              for (int j = 0; j < curvature_region; j++) {
                point_picked[i + 1 + j] = true;
              }
            }
          }
        }

        float diffPrevious =
            (point.getVector3fMap() - previousPoint.getVector3fMap())
                .squaredNorm();
        float dis = point.getVector3fMap().squaredNorm();

        // this will reject a point if the difference in distance
        // or depth of point and its neighbors is outside a bound i.e.
        // rejecting very sharp ramp like points
        if (diffNext > 0.0002 * dis && diffPrevious > 0.0002 * dis) {
          point_picked[i] = true;
        }
      }

      if (region_idx == feature_regions) {
        sector_end = total_points - 1;
      }
      CurvaturePairs sub_cloud_curvatures(
          cloud_curvatures.begin() + sector_start,
          cloud_curvatures.begin() + sector_end);

      extractFeaturesFromSector(
          scan_lines[line_idx], sub_cloud_curvatures, source_edges,
          source_surfaces, &point_picked);
    }
  }
}

void LoamAlignment::extractFeaturesFromSector(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr& points_in,
    const CurvaturePairs& cloud_curvatures,
    pcl::PointCloud<pcl::PointXYZI>::Ptr edges,
    pcl::PointCloud<pcl::PointXYZI>::Ptr surfaces,
    std::vector<bool>* point_picked) {
  CurvaturePairs cloud_curvatures_sorted = cloud_curvatures;
  std::sort(
      cloud_curvatures_sorted.begin(), cloud_curvatures_sorted.end(),
      [](const CurvaturePair& a, const CurvaturePair& b) {
        return a.second < b.second;
      });

  int largest_picked_num = 0;

  int point_info_count = 0;
  for (int i = cloud_curvatures_sorted.size() - 1; i >= 0; i--) {
    int ind = cloud_curvatures_sorted[i].first;
    if (!(*point_picked)[ind]) {
      if (cloud_curvatures_sorted[i].second <= 0.1) {
        break;
      }

      largest_picked_num++;
      (*point_picked)[ind] = true;

      if (largest_picked_num <= 2) {
        edges->push_back(points_in->points[ind]);
        point_info_count++;

        for (int k = 1; k <= 5; k++) {
          const Eigen::Vector3f point_diff =
              points_in->points[ind + k].getVector3fMap() -
              points_in->points[ind + k - 1].getVector3fMap();
          if (point_diff.norm() > sqrt(0.05)) {
            break;
          }
          (*point_picked)[ind + k] = true;
        }

        for (int k = -1; k >= -5; k--) {
          const Eigen::Vector3f point_diff =
              points_in->points[ind + k].getVector3fMap() -
              points_in->points[ind + k + 1].getVector3fMap();
          if (point_diff.norm() > sqrt(0.05)) {
            break;
          }
          (*point_picked)[ind + k] = true;
        }
      } else {
        break;
      }
    }
  }

  int smallest_picked_num = 0;

  for (int i = 0; i <= cloud_curvatures_sorted.size() - 1u; i++) {
    if (cloud_curvatures_sorted[i].second > 0.1)
      break;
    int ind = cloud_curvatures_sorted[i].first;
    if (smallest_picked_num <= 4) {
      if (!(*point_picked)[ind]) {
        surfaces->push_back(points_in->points[ind]);
        smallest_picked_num++;

        for (int k = 1; k <= 5; k++) {
          const Eigen::Vector3f point_diff =
              points_in->points[ind + k].getVector3fMap() -
              points_in->points[ind + k - 1].getVector3fMap();
          if (point_diff.norm() > sqrt(0.05)) {
            break;
          }
          (*point_picked)[ind + k] = true;
        }

        for (int k = -1; k >= -5; k--) {
          const Eigen::Vector3f point_diff =
              points_in->points[ind + k].getVector3fMap() -
              points_in->points[ind + k + 1].getVector3fMap();
          if (point_diff.norm() > sqrt(0.05)) {
            break;
          }
          (*point_picked)[ind + k] = true;
        }
      }
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
    const PclPointCloudPtr<pcl::PointXYZI>& target_edges,
    const PclPointCloudPtr<pcl::PointXYZI>& source_edges,
    const aslam::Transformation& T_target_source,
    ceres::Problem* problem, ceres::LossFunction *loss_function) {
  CHECK_NOTNULL(problem);
  CHECK_NOTNULL(loss_function);

  size_t n_corners = 0u;

  PclPointCloudPtr<pcl::PointXYZI> source_edges_transformed(
      new pcl::PointCloud<pcl::PointXYZI>);
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
    LOG(WARNING) << "Not enough edge points matched in Loam Alignment";
    LOG(WARNING) << n_corners;
  }
}

void LoamAlignment::addSurfaceCostFactors(
    const PclPointCloudPtr<pcl::PointXYZI>& target_surfaces,
    const PclPointCloudPtr<pcl::PointXYZI>& source_surfaces,
    const aslam::Transformation& T_target_source,
    ceres::Problem* problem, ceres::LossFunction *loss_function) {
  CHECK_NOTNULL(problem);
  CHECK_NOTNULL(loss_function);

  size_t n_surfaces = 0;

  PclPointCloudPtr<pcl::PointXYZI> source_surfaces_transformed(
      new pcl::PointCloud<pcl::PointXYZI>);
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
    LOG(WARNING) << "Not enough surface points matched in Loam Alignment";
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
