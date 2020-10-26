#ifndef REGISTRATION_TOOLBOX_ALIGNMENT_LOAM_ALIGNMENT_H_
#define REGISTRATION_TOOLBOX_ALIGNMENT_LOAM_ALIGNMENT_H_

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <ceres/ceres.h>
#include <ceres/rotation.h>


#include "registration-toolbox/alignment/base-alignment.h"
#include "registration-toolbox/common/registration-gflags.h"

namespace regbox {

template <typename T>
using PclPointCloudPtr = typename boost::shared_ptr<pcl::PointCloud<T>>;

class LoamAlignment : public BaseAlignment<PclPointCloudPtr<pcl::PointXYZI>> {
 public:
  virtual ~LoamAlignment() = default;

 protected:
  RegistrationResult registerCloudImpl(
      const PclPointCloudPtr<pcl::PointXYZI>& target,
      const PclPointCloudPtr<pcl::PointXYZI>& source,
      const aslam::Transformation& prior_T_target_source) override;

 private:
  void extractFeaturesFromInputClouds(
      const PclPointCloudPtr<pcl::PointXYZI>& target,
      const PclPointCloudPtr<pcl::PointXYZI>& source);

  void addEdgeCostFactors(
      const PclPointCloudPtr<pcl::PointXYZI>& target_edges,
      const PclPointCloudPtr<pcl::PointXYZI>& source_edges,
      const aslam::Transformation& T_target_source, ceres::Problem* problem,
      ceres::LossFunction* loss_function);

  void addSurfaceCostFactors(
      const PclPointCloudPtr<pcl::PointXYZI>& target_surfaces,
      const PclPointCloudPtr<pcl::PointXYZI>& source_surfaces,
      const aslam::Transformation& T_target_source, ceres::Problem* problem,
      ceres::LossFunction* loss_function);

  pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr kd_tree_target_edges_;
  pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr kd_tree_target_surfaces_;
  PclPointCloudPtr<pcl::PointXYZI> target_edges_;
  PclPointCloudPtr<pcl::PointXYZI> target_surfaces_;
  PclPointCloudPtr<pcl::PointXYZI> source_surfaces_;
  PclPointCloudPtr<pcl::PointXYZI> source_edges_;

  double parameters_[7] = {0, 0, 0, 1, 0, 0, 0};
};

class SurfaceAnalyticCostFunction : public ceres::SizedCostFunction<1, 7> {
 public:
  SurfaceAnalyticCostFunction(
      Eigen::Vector3d curr_point, Eigen::Vector3d plane_unit_norm,
      double negative_OA_dot_norm)
      : curr_point_(curr_point),
        plane_unit_norm_(plane_unit_norm),
        negative_OA_dot_norm_(negative_OA_dot_norm) {}

  virtual ~SurfaceAnalyticCostFunction() {}

  virtual bool Evaluate(
      double const* const* parameters, double* residuals,
      double** jacobians) const;

  Eigen::Vector3d curr_point_;
  Eigen::Vector3d plane_unit_norm_;
  double negative_OA_dot_norm_;
};

class EdgeAnalyticCostFunction : public ceres::SizedCostFunction<3, 7> {
 public:
  EdgeAnalyticCostFunction(
      Eigen::Vector3d curr_point, Eigen::Vector3d last_point_a,
      Eigen::Vector3d last_point_b)
      : curr_point_(curr_point),
        last_point_a_(last_point_a),
        last_point_b_(last_point_b) {}
  virtual ~EdgeAnalyticCostFunction() {}
  virtual bool Evaluate(
      double const* const* parameters, double* residuals,
      double** jacobians) const;

  Eigen::Vector3d curr_point_;
  Eigen::Vector3d last_point_a_;
  Eigen::Vector3d last_point_b_;
};

class PoseSE3Parameterization : public ceres::LocalParameterization {
 public:
  PoseSE3Parameterization() {}

  virtual ~PoseSE3Parameterization() {}

  bool Plus(const double* x, const double* delta, double* x_plus_delta) const;

  bool ComputeJacobian(const double* x, double* jacobian) const;

  int GlobalSize() const {
    return 7;
  }

  int LocalSize() const {
    return 6;
  }

 private:
  void getTransformFromSe3(
      const Eigen::Matrix<double, 6, 1>& se3, Eigen::Quaterniond* q,
      Eigen::Vector3d* t) const;
};

Eigen::Matrix3d skew(const Eigen::Vector3d& mat_in);

}  // namespace regbox

#endif  // REGISTRATION_TOOLBOX_ALIGNMENT_LOAM_ALIGNMENT_H_
