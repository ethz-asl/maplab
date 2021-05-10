#ifndef REGISTRATION_TOOLBOX_LOAM_FEATURE_DETECTOR_H_
#define REGISTRATION_TOOLBOX_LOAM_FEATURE_DETECTOR_H_

#include <utility>
#include <vector>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <resources-common/point-cloud.h>

template <typename T>
using PclPointCloudPtr = typename boost::shared_ptr<pcl::PointCloud<T>>;
using CurvaturePair = typename std::pair<int, float>;
using CurvaturePairs = typename std::vector<std::pair<int, float>>;

namespace regbox {

class LoamFeatureDetector {
 public:
  LoamFeatureDetector();
  ~LoamFeatureDetector() = default;

  void extractLoamFeaturesFromPointCloud(
      const resources::PointCloud& point_cloud,
      resources::PointCloud* feature_cloud, size_t* n_edges = nullptr,
      size_t* n_surfaces = nullptr);

 private:
  void extractFeaturesFromScanLine(
      const pcl::PointCloud<pcl::PointXYZ>::Ptr& scan_line,
      pcl::PointCloud<pcl::PointXYZ>::Ptr edges,
      pcl::PointCloud<pcl::PointXYZ>::Ptr surfaces);

  void downSampleFeatures(
      pcl::PointCloud<pcl::PointXYZ>::Ptr edges,
      pcl::PointCloud<pcl::PointXYZ>::Ptr surfaces);

  void markUnstablePointsAsPicked(
      const pcl::PointCloud<pcl::PointXYZ>::Ptr& scan_line,
      std::vector<bool>* point_picked);

  void markCurvatureRegionAsPicked(
      const int& point_idx, std::vector<bool>* point_picked);

  void markFirstHalfCurvatureRegionAsPicked(
      const int& point_idx, std::vector<bool>* point_picked);
  void markSecondHalfCurvatureRegionAsPicked(
      const int& point_idx, std::vector<bool>* point_picked);

  void markCurvatureRegionAsPicked(
      const int& point_idx, const double& distance_threshold_m,
      const pcl::PointCloud<pcl::PointXYZ>::Ptr& scan_line,
      std::vector<bool>* point_picked);

  void calculateCurvatures(
      const pcl::PointCloud<pcl::PointXYZ>::Ptr& scan_line,
      CurvaturePairs* curvatures);
  pcl::PointCloud<pcl::PointXYZ> pickedpoints_;
  float max_angle_between_valid_neighbors_rad_;
  static constexpr float kDistanceEpsillon = 0.0001;
  static constexpr float kMaxCurvatureRegionNeighborDistanceSquaredm2 = 0.05;
  static constexpr float kSquaredPointToNextPointDistancem2 = 0.1;
  static constexpr float kRampFactor = 0.0002;
  static constexpr float kWeightedDistanceThreshold = 0.1;
};

}  // namespace regbox

#endif  // REGISTRATION_TOOLBOX_LOAM_FEATURE_DETECTOR_H_
