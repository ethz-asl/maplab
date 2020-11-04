#ifndef REGISTRATION_TOOLBOX_ALIGNMENT_LOAM_ALIGNMENT_H_
#define REGISTRATION_TOOLBOX_ALIGNMENT_LOAM_ALIGNMENT_H_

#include <utility>
#include <vector>

#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "registration-toolbox/alignment/base-alignment.h"
#include "registration-toolbox/common/registration-gflags.h"

namespace regbox {

template <typename T>
using PclPointCloudPtr = typename boost::shared_ptr<pcl::PointCloud<T>>;
using CurvaturePair = typename std::pair<size_t, double>;
using CurvaturePairs = typename std::vector<std::pair<size_t, double>>;

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

  void extractLoamFeaturesFromPointCloud(
      const PclPointCloudPtr<pcl::PointXYZI>& point_cloud,
      PclPointCloudPtr<pcl::PointXYZI> edges,
      PclPointCloudPtr<pcl::PointXYZI> surfaces);

  void extractFeaturesFromScanLine(
      const pcl::PointCloud<pcl::PointXYZI>::Ptr& scan_line,
      pcl::PointCloud<pcl::PointXYZI>::Ptr edges,
      pcl::PointCloud<pcl::PointXYZI>::Ptr surfaces);

  void extractFeaturesFromFeatureRegion(
      const pcl::PointCloud<pcl::PointXYZI>::Ptr& points_in,
      const CurvaturePairs& cloud_curvatures,
      pcl::PointCloud<pcl::PointXYZI>::Ptr edges,
      pcl::PointCloud<pcl::PointXYZI>::Ptr surfaces,
      std::vector<bool>* point_picked);

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

  bool calculateSolutionCovariance(
      ceres::Problem* problem, Eigen::Matrix<float, 7, 7>* covariance);

  void downSampleFeatures(
      pcl::PointCloud<pcl::PointXYZI>::Ptr edges,
      pcl::PointCloud<pcl::PointXYZI>::Ptr surfaces);

  void markUnstablePointsAsPicked(
      const pcl::PointCloud<pcl::PointXYZI>::Ptr& scan_line,
      std::vector<bool>* point_picked);

  void markCurvatureRegionAsPicked(
      const int& point_idx, std::vector<bool>* point_picked);

  void markFirstHalfCurvatureRegionAsPicked(
      const int& point_idx, std::vector<bool>* point_picked);
  void markSecondHalfCurvatureRegionAsPicked(
      const int& point_idx, std::vector<bool>* point_picked);

  void markCurvatureRegionAsPicked(
      const int& point_idx, const double& distance_threshold_m,
      const pcl::PointCloud<pcl::PointXYZI>::Ptr& scan_line,
      std::vector<bool>* point_picked);

  void calculateCurvatures(
      const pcl::PointCloud<pcl::PointXYZI>::Ptr& scan_line,
      CurvaturePairs* curvatures);

  pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr kd_tree_target_edges_;
  pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr kd_tree_target_surfaces_;
  PclPointCloudPtr<pcl::PointXYZI> target_edges_;
  PclPointCloudPtr<pcl::PointXYZI> target_surfaces_;
  PclPointCloudPtr<pcl::PointXYZI> source_surfaces_;
  PclPointCloudPtr<pcl::PointXYZI> source_edges_;
  double parameters_[7];
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
