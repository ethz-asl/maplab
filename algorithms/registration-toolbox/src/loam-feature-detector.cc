#include "registration-toolbox/loam-feature-detector.h"

#include <gflags/gflags.h>
#include <glog/logging.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization/rviz-visualization-sink.h>

DEFINE_int32(
    regbox_loam_curvature_region, 5,
    "Number of points on each side for curvature calculation");
DEFINE_int32(
    regbox_loam_feature_regions, 6,
    "Number of feature regions that ring is split into");
DEFINE_int32(
    regbox_loam_edges_per_region, 2, "Number of edges per feature region");
DEFINE_int32(
    regbox_loam_surfaces_per_region, 4, "Number of edges per feature region");
DEFINE_double(
    regbox_loam_edge_filter_size_m, 0.2,
    "Voxel Grid Filter size for edge feature downsampling");
DEFINE_double(
    regbox_loam_surface_filter_size_m, 0.4,
    "Voxel Grid Filter size for surface feature downsampling");
DEFINE_double(
    regbox_loam_surface_curvature_threshold, 0.1,
    "Curvature below which a point is considered surface");

namespace regbox {

void LoamFeatureDetector::extractLoamFeaturesFromPointCloud(
    const resources::PointCloud& point_cloud,
    resources::PointCloud* feature_cloud) {
  CHECK(point_cloud.hasRings());
  CHECK_NOTNULL(feature_cloud);

  PclPointCloudPtr<pcl::PointXYZ> edges(new pcl::PointCloud<pcl::PointXYZ>);
  PclPointCloudPtr<pcl::PointXYZ> surfaces(new pcl::PointCloud<pcl::PointXYZ>);

  std::vector<int> indices;

  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> scan_lines;
  pickedpoints_.clear();

  pcl::PointCloud<pcl::PointXYZ> loam_input;
  uint32_t max_ring =
      *std::max_element(point_cloud.rings.begin(), point_cloud.rings.end());

  if (max_ring > 16) {
    LOG(WARNING) << "MAX RING: " << max_ring;
    return;
  }
  for (size_t idx = 0u; idx <= max_ring; idx++) {
    scan_lines.push_back(
        PclPointCloudPtr<pcl::PointXYZ>(new pcl::PointCloud<pcl::PointXYZ>));
  }
  for (int idx = 0u; idx < point_cloud.size(); idx++) {
    CHECK_GE(point_cloud.rings[idx], 0);
    pcl::PointXYZ point;
    point.x = point_cloud.xyz[3u * idx + 0];
    point.y = point_cloud.xyz[3u * idx + 1];
    point.z = point_cloud.xyz[3u * idx + 2];
    const double planar_distance = sqrt(point.x * point.x + point.y * point.y);
    if (abs(point.z) < 0.001)
      continue;
    if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
        !std::isfinite(point.z)) {
      continue;
    }
    scan_lines[static_cast<size_t>(point_cloud.rings[idx])]->push_back(point);
    loam_input.push_back(point);
  }
  for (const pcl::PointCloud<pcl::PointXYZ>::Ptr& scan_line : scan_lines) {
    if (scan_line->size() >= 2 * FLAGS_regbox_loam_curvature_region + 1) {
      extractFeaturesFromScanLine(scan_line, edges, surfaces);
    }
  }

  downSampleFeatures(edges, surfaces);
  *feature_cloud = resources::PointCloud();
  for (const pcl::PointXYZ& point : *edges) {
    feature_cloud->xyz.push_back(point.x);
    feature_cloud->xyz.push_back(point.y);
    feature_cloud->xyz.push_back(point.z);
    feature_cloud->labels.push_back(1u);
  }
  for (const pcl::PointXYZ& point : *surfaces) {
    feature_cloud->xyz.push_back(point.x);
    feature_cloud->xyz.push_back(point.y);
    feature_cloud->xyz.push_back(point.z);
    feature_cloud->labels.push_back(0u);
  }
}

void LoamFeatureDetector::extractFeaturesFromScanLine(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& scan_line,
    pcl::PointCloud<pcl::PointXYZ>::Ptr edges,
    pcl::PointCloud<pcl::PointXYZ>::Ptr surfaces) {
  std::vector<bool> point_picked(scan_line->size(), false);
  markUnstablePointsAsPicked(scan_line, &point_picked);
  CurvaturePairs cloud_curvatures;
  calculateCurvatures(scan_line, &cloud_curvatures);
  for (size_t idx = 0u; idx < cloud_curvatures.size(); ++idx) {
    if (point_picked[cloud_curvatures[idx].first]) {
      pcl::PointXYZ point = (*scan_line)[cloud_curvatures[idx].first];
      pickedpoints_.push_back(point);
    }
  }

  std::sort(
      cloud_curvatures.begin(), cloud_curvatures.end(),
      [](const CurvaturePair& a, const CurvaturePair& b) {
        return a.second < b.second;
      });

  std::vector<int> edges_extracted_per_region(
      FLAGS_regbox_loam_feature_regions, 0);
  std::vector<int> surfaces_extracted_per_region(
      FLAGS_regbox_loam_feature_regions, 0);

  for (int idx = cloud_curvatures.size() - 1; idx >= 0; idx--) {
    int ind = cloud_curvatures[idx].first;
    if (!point_picked[ind]) {
      if (cloud_curvatures[idx].second <= 0.1) {
        break;
      }
      if (edges_extracted_per_region
              [FLAGS_regbox_loam_feature_regions * ind / scan_line->size()] <
          FLAGS_regbox_loam_edges_per_region) {
        point_picked[ind] = true;
        edges_extracted_per_region
            [FLAGS_regbox_loam_feature_regions * ind / scan_line->size()]++;
        edges->push_back((*scan_line)[ind]);
        markCurvatureRegionAsPicked(ind, sqrt(0.05), scan_line, &point_picked);
      } else {
        break;
      }
    }
  }

  for (int idx = 0; idx < cloud_curvatures.size(); idx++) {
    int ind = cloud_curvatures[idx].first;
    if (!point_picked[ind]) {
      if (cloud_curvatures[idx].second > 0.1) {
        break;
      }
      if (surfaces_extracted_per_region
              [FLAGS_regbox_loam_feature_regions * ind / scan_line->size()] <
          FLAGS_regbox_loam_surfaces_per_region) {
        surfaces->push_back((*scan_line)[ind]);
        surfaces_extracted_per_region
            [FLAGS_regbox_loam_feature_regions * ind / scan_line->size()]++;
        markCurvatureRegionAsPicked(ind, sqrt(0.05), scan_line, &point_picked);
      }
    }
  }
}

void LoamFeatureDetector::markUnstablePointsAsPicked(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& scan_line,
    std::vector<bool>* point_picked) {
  for (int point_idx = FLAGS_regbox_loam_curvature_region;
       point_idx < scan_line->size() - FLAGS_regbox_loam_curvature_region;
       point_idx++) {
    const pcl::PointXYZ& point = (*scan_line)[point_idx];
    const pcl::PointXYZ& previous_point = (*scan_line)[point_idx - 1];
    float alpha = acos(
        point.getVector3fMap().dot(previous_point.getVector3fMap()) /
        (point.getVector3fMap().norm() *
         previous_point.getVector3fMap().norm()));
    if (abs(alpha) > 1. * M_PI / 180.) {
      markCurvatureRegionAsPicked(point_idx, point_picked);
    }
    const float point_distance_m = point.getVector3fMap().norm();

    const pcl::PointXYZ& next_point = (*scan_line)[point_idx + 1];

    const float point_to_next_point_distance_m =
        (next_point.getVector3fMap() - point.getVector3fMap()).norm();
    const float squared_point_to_next_point_distance_m2 =
        (point.getVector3fMap() - next_point.getVector3fMap()).squaredNorm();

    if (squared_point_to_next_point_distance_m2 > 0.1) {
      const float next_point_distance_m = next_point.getVector3fMap().norm();

      if (point_distance_m > next_point_distance_m) {
        float weighted_distance = (next_point.getVector3fMap() -
                                   (next_point_distance_m / point_distance_m) *
                                       point.getVector3fMap())
                                      .norm() /
                                  next_point_distance_m;

        if (weighted_distance < 0.1) {
          markFirstHalfCurvatureRegionAsPicked(point_idx, point_picked);
          continue;
        }
      } else {
        float weighted_distance = (point.getVector3fMap() -
                                   (point_distance_m / next_point_distance_m) *
                                       next_point.getVector3fMap())
                                      .norm() /
                                  point_distance_m;

        if (weighted_distance < 0.1) {
          markSecondHalfCurvatureRegionAsPicked(point_idx, point_picked);
        }
      }
    }

    float squared_point_to_previous_point_distance_m2 =
        (point.getVector3fMap() - previous_point.getVector3fMap())
            .squaredNorm();
    float squared_point_distance_m2 = point.getVector3fMap().squaredNorm();

    // this will reject a point if the difference in distance of point and its
    // neighbors is outside a bound i.e. rejecting very sharp ramp like points
    const float squared_point_distance_threshold_m2 =
        0.0002 * squared_point_distance_m2;
    if (squared_point_to_next_point_distance_m2 >
            squared_point_distance_threshold_m2 &&
        squared_point_to_previous_point_distance_m2 >
            squared_point_distance_threshold_m2) {
      (*point_picked)[point_idx] = true;
    }
  }
}

void LoamFeatureDetector::markCurvatureRegionAsPicked(
    const int& point_idx, std::vector<bool>* point_picked) {
  for (int neighbor_idx = -FLAGS_regbox_loam_curvature_region;
       neighbor_idx <= FLAGS_regbox_loam_curvature_region; neighbor_idx++) {
    (*point_picked)[point_idx + neighbor_idx] = true;
  }
}

void LoamFeatureDetector::markCurvatureRegionAsPicked(
    const int& point_idx, const double& distance_threshold_m,
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& scan_line,
    std::vector<bool>* point_picked) {
  (*point_picked)[point_idx] = true;
  for (int neighbor_idx = 1; neighbor_idx <= FLAGS_regbox_loam_curvature_region;
       neighbor_idx++) {
    const float point_to_neighbor_distance_m =
        ((*scan_line)[point_idx + neighbor_idx].getVector3fMap() -
         (*scan_line)[point_idx + neighbor_idx - 1].getVector3fMap())
            .norm();
    if (point_to_neighbor_distance_m > distance_threshold_m) {
      break;
    }
    (*point_picked)[point_idx + neighbor_idx] = true;
  }

  for (int neighbor_idx = 1; neighbor_idx <= FLAGS_regbox_loam_curvature_region;
       neighbor_idx++) {
    const float point_to_neighbor_distance_m =
        ((*scan_line)[point_idx - neighbor_idx].getVector3fMap() -
         (*scan_line)[point_idx - neighbor_idx + 1].getVector3fMap())
            .norm();
    if (point_to_neighbor_distance_m > distance_threshold_m) {
      break;
    }
    (*point_picked)[point_idx + neighbor_idx] = true;
  }
}

void LoamFeatureDetector::markFirstHalfCurvatureRegionAsPicked(
    const int& point_idx, std::vector<bool>* point_picked) {
  for (int neighbor_idx = 0; neighbor_idx <= FLAGS_regbox_loam_curvature_region;
       neighbor_idx++) {
    (*point_picked)[point_idx - neighbor_idx] = true;
  }
}

void LoamFeatureDetector::markSecondHalfCurvatureRegionAsPicked(
    const int& point_idx, std::vector<bool>* point_picked) {
  (*point_picked)[point_idx] = true;
  for (int neighbor_idx = 1; neighbor_idx <= FLAGS_regbox_loam_curvature_region;
       neighbor_idx++) {
    (*point_picked)[point_idx + neighbor_idx] = true;
  }
}

void LoamFeatureDetector::downSampleFeatures(
    pcl::PointCloud<pcl::PointXYZ>::Ptr edges,
    pcl::PointCloud<pcl::PointXYZ>::Ptr surfaces) {
  pcl::VoxelGrid<pcl::PointXYZ> edge_filter;
  edge_filter.setInputCloud(edges);
  edge_filter.setLeafSize(
      FLAGS_regbox_loam_edge_filter_size_m,
      FLAGS_regbox_loam_edge_filter_size_m,
      FLAGS_regbox_loam_edge_filter_size_m);
  edge_filter.filter(*edges);

  pcl::VoxelGrid<pcl::PointXYZ> surface_filter;
  surface_filter.setInputCloud(surfaces);
  surface_filter.setLeafSize(
      FLAGS_regbox_loam_surface_filter_size_m,
      FLAGS_regbox_loam_surface_filter_size_m,
      FLAGS_regbox_loam_surface_filter_size_m);
  surface_filter.filter(*surfaces);
}

void LoamFeatureDetector::calculateCurvatures(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& scan_line,
    CurvaturePairs* curvatures) {
  for (int point_idx = FLAGS_regbox_loam_curvature_region;
       point_idx < (*scan_line).size() - FLAGS_regbox_loam_curvature_region;
       point_idx++) {
    Eigen::Vector3f merged_point = -2. * FLAGS_regbox_loam_curvature_region *
                                   (*scan_line)[point_idx].getVector3fMap();
    for (int neighbor_idx = 1;
         neighbor_idx <= FLAGS_regbox_loam_curvature_region; neighbor_idx++) {
      merged_point += (*scan_line)[point_idx + neighbor_idx].getVector3fMap();
      merged_point += (*scan_line)[point_idx - neighbor_idx].getVector3fMap();
    }
    const float curvature =
        merged_point.norm() / (2 * FLAGS_regbox_loam_curvature_region *
                               (*scan_line)[point_idx].getVector3fMap().norm());
    const CurvaturePair curvature_pair(point_idx, merged_point.norm());
    curvatures->push_back(curvature_pair);
  }
}

}  // namespace regbox
