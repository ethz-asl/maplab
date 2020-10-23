#include "registration-toolbox/alignment/loam-alignment.h"

#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/passthrough.h>

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
  kd_tree_target_edges_->setInputCloud(target_edges_);
  kd_tree_target_surfaces_->setInputCloud(target_surfaces_);
  int optimization_count = 100;

  Eigen::Map<Eigen::Quaterniond> q_w_curr =
        Eigen::Map<Eigen::Quaterniond>(&parameters_[0]);
  Eigen::Map<Eigen::Vector3d> t_w_curr =
      Eigen::Map<Eigen::Vector3d>(&parameters_[0] + 4);
  q_w_curr = Eigen::Quaterniond(prior_T_target_source.getEigenQuaternion());
  t_w_curr = prior_T_target_source.getPosition();

  for (int iterCount = 0; iterCount < optimization_count; iterCount++){

    ceres::LossFunction *loss_function = new ceres::HuberLoss(0.1);
    ceres::Problem::Options problem_options;
    ceres::Problem problem(problem_options);
    problem.AddParameterBlock(parameters_, 7, new PoseSE3Parameterization());

    const aslam::Transformation estimated_T_target_source(q_w_curr, t_w_curr);

    addEdgeCostFactors(target_edges_, source_edges_,
      estimated_T_target_source, &problem, loss_function);
    addSurfaceCostFactors(target_surfaces_,source_surfaces_,
      estimated_T_target_source, &problem, loss_function);

    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR;
    options.max_num_iterations = 10;
    options.minimizer_progress_to_stdout = false;
    options.check_gradients = false;
    options.gradient_check_relative_precision = 1e-4;
    ceres::Solver::Summary summary;

    ceres::Solve(options, &problem, &summary);
  }

  const aslam::Transformation T_target_source(q_w_curr, t_w_curr);

  PclPointCloudPtr<pcl::PointXYZI> reg(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::transformPointCloud(*source, *reg,
    T_target_source.getTransformationMatrix());
  const Eigen::MatrixXd cov = Eigen::MatrixXd::Identity(6, 6) * 1e-4;

  return RegistrationResult(reg, cov, T_target_source, true);
}

void LoamAlignment::extractFeaturesFromInputClouds(
    const PclPointCloudPtr<pcl::PointXYZI>& target,
    const PclPointCloudPtr<pcl::PointXYZI>& source) {

  target_surfaces_ = PclPointCloudPtr<pcl::PointXYZI>(
      new pcl::PointCloud<pcl::PointXYZI>);
  target_edges_ = PclPointCloudPtr<pcl::PointXYZI>(
      new pcl::PointCloud<pcl::PointXYZI>);

  for (pcl::PointXYZI point : target->points) {
    if(point.intensity==0) {
      target_surfaces_->push_back(point);
    } else if (point.intensity==1) {
      target_edges_->push_back(point);
    }
  }

  source_surfaces_ = PclPointCloudPtr<pcl::PointXYZI>(
      new pcl::PointCloud<pcl::PointXYZI>);
  source_edges_ = PclPointCloudPtr<pcl::PointXYZI>(
      new pcl::PointCloud<pcl::PointXYZI>);

  for (pcl::PointXYZI point : source->points) {
    if(point.intensity==0) {
      source_surfaces_->push_back(point);
    } else if (point.intensity==1) {
      source_edges_->push_back(point);
    }
  }

}

bool EdgeAnalyticCostFunction::Evaluate(
  double const *const *parameters,
  double *residuals,
  double **jacobians) const {

    Eigen::Map<const Eigen::Quaterniond> q_last_curr(parameters[0]);
    Eigen::Map<const Eigen::Vector3d> t_last_curr(parameters[0] + 4);
    Eigen::Vector3d lp;
    lp = q_last_curr * curr_point_ + t_last_curr; //new point
    Eigen::Vector3d nu = (lp - last_point_a_).cross(lp - last_point_b_);
    Eigen::Vector3d de = last_point_a_ - last_point_b_;

    residuals[0] = nu.x() / de.norm();
    residuals[1] = nu.y() / de.norm();
    residuals[2] = nu.z() / de.norm();

    if(jacobians != NULL)
    {
      if(jacobians[0] != NULL)
      {
        Eigen::Matrix3d skew_lp = skew(lp);
        Eigen::Matrix<double, 3, 6> dp_by_so3;
        dp_by_so3.block<3,3>(0,0) = -skew_lp;
        (dp_by_so3.block<3,3>(0, 3)).setIdentity();
        Eigen::Map<Eigen::Matrix<double, 3, 7, Eigen::RowMajor>> J_se3(
          jacobians[0]);
        J_se3.setZero();
        Eigen::Vector3d re = last_point_b_ - last_point_a_;
        Eigen::Matrix3d skew_re = skew(re);

        J_se3.block<3,6>(0,0) = skew_re * dp_by_so3/de.norm();
      }
    }

    return true;

}

bool SurfaceAnalyticCostFunction::Evaluate(
  double const *const *parameters,
  double *residuals,
  double **jacobians) const {

    Eigen::Map<const Eigen::Quaterniond> q_w_curr(parameters[0]);
    Eigen::Map<const Eigen::Vector3d> t_w_curr(parameters[0] + 4);
    Eigen::Vector3d point_w = q_w_curr * curr_point_ + t_w_curr;

    residuals[0] = plane_unit_norm_.dot(point_w) + negative_OA_dot_norm_;

    if(jacobians != NULL) {
      if(jacobians[0] != NULL) {
        Eigen::Matrix3d skew_point_w = skew(point_w);
        Eigen::Matrix<double, 3, 6> dp_by_so3;
        dp_by_so3.block<3,3>(0,0) = -skew_point_w;
        (dp_by_so3.block<3,3>(0, 3)).setIdentity();
        Eigen::Map<Eigen::Matrix<double, 1, 7, Eigen::RowMajor>> J_se3(
          jacobians[0]);
        J_se3.setZero();
        J_se3.block<1,6>(0,0) = plane_unit_norm_.transpose() * dp_by_so3;

      }
    }
    return true;

}


bool PoseSE3Parameterization::Plus(
    const double *x,
    const double *delta,
    double *x_plus_delta) const {
    Eigen::Map<const Eigen::Vector3d> trans(x + 4);

    Eigen::Quaterniond delta_q;
    Eigen::Vector3d delta_t;
    getTransformFromSe3(
      Eigen::Map<const Eigen::Matrix<double,6,1>>(delta), &delta_q, &delta_t);
    Eigen::Map<const Eigen::Quaterniond> quater(x);
    Eigen::Map<Eigen::Quaterniond> quater_plus(x_plus_delta);
    Eigen::Map<Eigen::Vector3d> trans_plus(x_plus_delta + 4);

    quater_plus = delta_q * quater;
    trans_plus = delta_q * trans + delta_t;

    return true;
}


bool PoseSE3Parameterization::ComputeJacobian(
    const double *x,
    double *jacobian) const {
  Eigen::Map<Eigen::Matrix<double, 7, 6, Eigen::RowMajor>> j(jacobian);
  (j.topRows(6)).setIdentity();
  (j.bottomRows(1)).setZero();

  return true;
}


void PoseSE3Parameterization::getTransformFromSe3(
    const Eigen::Matrix<double,6,1>& se3,
    Eigen::Quaterniond* q,
    Eigen::Vector3d* t) const {

  Eigen::Vector3d omega(se3.data());
  Eigen::Vector3d upsilon(se3.data()+3);
  Eigen::Matrix3d Omega = skew(omega);

  double theta = omega.norm();
  double half_theta = 0.5*theta;

  double imag_factor;
  double real_factor = cos(half_theta);
  if(theta<1e-10)
  {
      double theta_sq = theta*theta;
      double theta_po4 = theta_sq*theta_sq;
      imag_factor = 0.5-0.0208333*theta_sq+0.000260417*theta_po4;
  }
  else
  {
      double sin_half_theta = sin(half_theta);
      imag_factor = sin_half_theta/theta;
  }

  *q = Eigen::Quaterniond(real_factor, imag_factor*omega.x(),
    imag_factor*omega.y(), imag_factor*omega.z());


  Eigen::Matrix3d J;
  if (theta<1e-10) {
      J = q->matrix();
  } else {
    Eigen::Matrix3d Omega2 = Omega*Omega;
    J = (Eigen::Matrix3d::Identity() + (1-cos(theta))/(theta*theta)*Omega + (
      theta-sin(theta))/(pow(theta,3))*Omega2);
  }

  *t = J*upsilon;
}

void LoamAlignment::addEdgeCostFactors(
    const PclPointCloudPtr<pcl::PointXYZI>& target_edges,
    const PclPointCloudPtr<pcl::PointXYZI>& source_edges,
    const aslam::Transformation& T_target_source,
    ceres::Problem* problem, ceres::LossFunction *loss_function) {
    int corner_num=0;

    PclPointCloudPtr<pcl::PointXYZI> source_edges_transformed(
        new pcl::PointCloud<pcl::PointXYZI>);
    pcl::transformPointCloud(*source_edges,*source_edges_transformed,
      T_target_source.getTransformationMatrix());
    for (int i = 0; i < (int)source_edges->points.size(); i++)
    {

        std::vector<int> pointSearchInd;
        std::vector<float> pointSearchSqDis;
        kd_tree_target_edges_->nearestKSearch(
          source_edges_transformed->points[i], 5,
          pointSearchInd, pointSearchSqDis);
        if (pointSearchSqDis[4] < 1.0)
        {
            std::vector<Eigen::Vector3d> nearCorners;
            Eigen::Vector3d center(0, 0, 0);
            for (int j = 0; j < 5; j++) {
              Eigen::Vector3d tmp(target_edges->points[pointSearchInd[j]].x,
                                  target_edges->points[pointSearchInd[j]].y,
                                  target_edges->points[pointSearchInd[j]].z);
              center = center + tmp;
              nearCorners.push_back(tmp);
            }
            center = center / 5.0;

            Eigen::Matrix3d covMat = Eigen::Matrix3d::Zero();
            for (int j = 0; j < 5; j++) {
              Eigen::Matrix<double, 3, 1> tmpZeroMean = nearCorners[j] - center;
              covMat = covMat + tmpZeroMean * tmpZeroMean.transpose();
            }

            Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> saes(covMat);

            Eigen::Vector3d unit_direction = saes.eigenvectors().col(2);
            Eigen::Vector3d curr_point(source_edges->points[i].x,
              source_edges->points[i].y, source_edges->points[i].z);
            if (saes.eigenvalues()[2] > 3 * saes.eigenvalues()[1]) {
              Eigen::Vector3d point_on_line = center;
              Eigen::Vector3d point_a, point_b;
              point_a = 0.1 * unit_direction + point_on_line;
              point_b = -0.1 * unit_direction + point_on_line;

              ceres::CostFunction *cost_function = new EdgeAnalyticCostFunction(
                curr_point, point_a, point_b);
              problem->AddResidualBlock(cost_function, loss_function,
                parameters_);
              corner_num++;
            }
        }
    }
    if(corner_num<20){
        printf("not enough correct edge points");
    }

}

void LoamAlignment::addSurfaceCostFactors(
    const PclPointCloudPtr<pcl::PointXYZI>& target_surfaces,
    const PclPointCloudPtr<pcl::PointXYZI>& source_surfaces,
    const aslam::Transformation& T_target_source,
    ceres::Problem* problem, ceres::LossFunction *loss_function) {
  int surf_num=0;

  PclPointCloudPtr<pcl::PointXYZI> source_surfaces_transformed(
      new pcl::PointCloud<pcl::PointXYZI>);
  pcl::transformPointCloud(*source_surfaces,*source_surfaces_transformed,
     T_target_source.getTransformationMatrix());
  for (int i = 0; i < (int)source_surfaces->points.size(); i++)
  {
    std::vector<int> pointSearchInd;
    std::vector<float> pointSearchSqDis;
    kd_tree_target_surfaces_->nearestKSearch(
      source_surfaces_transformed->points[i], 5,
      pointSearchInd, pointSearchSqDis);

    Eigen::Matrix<double, 5, 3> matA0;
    Eigen::Matrix<double, 5, 1> matB0 = -Eigen::Matrix<double, 5, 1>::Ones();
    if (pointSearchSqDis[4] < 1.0)
    {

      for (int j = 0; j < 5; j++)
      {
        matA0(j, 0) = target_surfaces->points[pointSearchInd[j]].x;
        matA0(j, 1) = target_surfaces->points[pointSearchInd[j]].y;
        matA0(j, 2) = target_surfaces->points[pointSearchInd[j]].z;
      }
      // find the norm of plane
      Eigen::Vector3d norm = matA0.colPivHouseholderQr().solve(matB0);
      double negative_OA_dot_norm = 1 / norm.norm();
      norm.normalize();

      bool planeValid = true;
      for (int j = 0; j < 5; j++)
      {
        // if OX * n > 0.2, then plane is not fit well
        if (fabs(norm(0) * target_surfaces->points[pointSearchInd[j]].x +
                 norm(1) * target_surfaces->points[pointSearchInd[j]].y +
                 norm(2) * target_surfaces->points[pointSearchInd[j]].z +
                 negative_OA_dot_norm) > 0.2) {
          planeValid = false;
          break;
        }
      }
      Eigen::Vector3d curr_point(source_surfaces->points[i].x,
        source_surfaces->points[i].y, source_surfaces->points[i].z);
      if (planeValid)
      {
        ceres::CostFunction *cost_function = new SurfaceAnalyticCostFunction(
          curr_point, norm, negative_OA_dot_norm);
        problem->AddResidualBlock(cost_function, loss_function, parameters_);
        surf_num++;
      }
    }

  }
  if(surf_num<20){
      printf("not enough surface points");
  }
}

Eigen::Matrix<double,3,3> skew(
    const Eigen::Matrix<double,3,1>& mat_in) {
  Eigen::Matrix<double,3,3> skew_mat;
  skew_mat.setZero();
  skew_mat(0,1) = -mat_in(2);
  skew_mat(0,2) =  mat_in(1);
  skew_mat(1,2) = -mat_in(0);
  skew_mat(1,0) =  mat_in(2);
  skew_mat(2,0) = -mat_in(1);
  skew_mat(2,1) =  mat_in(0);
  return skew_mat;
}

}  // namespace regbox
