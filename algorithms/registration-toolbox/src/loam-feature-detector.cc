#include "registration-toolbox/loam-feature-detector.h"

#include <gflags/gflags.h>
#include <glog/logging.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>

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

namespace regbox {

void LoamFeatureDetector::extractLoamFeaturesFromPointCloud(
    const PclPointCloudPtr<pcl::PointXYZI>& point_cloud,
    PclPointCloudPtr<pcl::PointXYZI> feature_cloud) {
  CHECK_NOTNULL(point_cloud);
  CHECK_NOTNULL(feature_cloud);
  PclPointCloudPtr<pcl::PointXYZI> edges(new pcl::PointCloud<pcl::PointXYZI>);
  PclPointCloudPtr<pcl::PointXYZI> surfaces(
      new pcl::PointCloud<pcl::PointXYZI>);

  std::vector<int> indices;

  const int N_SCANS = 128;
  std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> scan_lines;

  for (int i = 0; i < N_SCANS; i++) {
    scan_lines.push_back(pcl::PointCloud<pcl::PointXYZI>::Ptr(
        new pcl::PointCloud<pcl::PointXYZI>));
  }
  int min_line = 1000;
  int max_line = 0;
  for (int idx = 0u; idx < point_cloud->size(); idx++) {
    const pcl::PointXYZI& point = point_cloud->points[idx];
    CHECK_GT(N_SCANS, point.intensity);
    CHECK_GE(point.intensity, 0);
    const double planar_distance = sqrt(point.x * point.x + point.y * point.y);
    // if (planar_distance < 0.001 || planar_distance > 57.)
    //   continue;
    // if (point.z == 0) continue;
    // if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
    //     !std::isfinite(point.z)) {
    //   continue;
    // }
    min_line = std::min(static_cast<int>(point.intensity), min_line);
    max_line = std::max(static_cast<int>(point.intensity), max_line);

    scan_lines[static_cast<int>(point.intensity)]->push_back(point);
  }
  for (const pcl::PointCloud<pcl::PointXYZI>::Ptr& scan_line : scan_lines) {
    if (scan_line->size() >= 2 * FLAGS_regbox_loam_curvature_region + 1) {
      extractFeaturesFromScanLine(scan_line, edges, surfaces);
    }
  }
  downSampleFeatures(edges, surfaces);
  feature_cloud->clear();
  for (pcl::PointXYZI point : *edges) {
    point.intensity = 1;
    feature_cloud->push_back(point);
  }
  for (pcl::PointXYZI point : *surfaces) {
    point.intensity = 0;
    feature_cloud->push_back(point);
  }
}

void LoamFeatureDetector::extractFeaturesFromScanLine(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr& scan_line,
    pcl::PointCloud<pcl::PointXYZI>::Ptr edges,
    pcl::PointCloud<pcl::PointXYZI>::Ptr surfaces) {
  std::vector<bool> point_picked(scan_line->size(), false);
  markUnstablePointsAsPicked(scan_line, &point_picked);
  CurvaturePairs cloud_curvatures;
  calculateCurvatures(scan_line, &cloud_curvatures);
  const int total_points =
      scan_line->size() - 2 * FLAGS_regbox_loam_curvature_region;

  const int sector_length = std::max(
      total_points / FLAGS_regbox_loam_feature_regions,
      2 * FLAGS_regbox_loam_curvature_region + 1);

  for (int region_idx = 0; region_idx < FLAGS_regbox_loam_feature_regions;
       region_idx++) {
    const int sector_start = sector_length * region_idx;
    int sector_end = sector_length * (region_idx + 1) - 1;

    if (sector_end >
        scan_line->size() - 2 * FLAGS_regbox_loam_curvature_region - 1) {
      break;
    }

    if (region_idx == FLAGS_regbox_loam_feature_regions) {
      sector_end = total_points - 1;
    }

    CurvaturePairs sub_cloud_curvatures(
        cloud_curvatures.begin() + sector_start,
        cloud_curvatures.begin() + sector_end);
    extractFeaturesFromFeatureRegion(
        scan_line, sub_cloud_curvatures, edges, surfaces, &point_picked);
  }
}

void LoamFeatureDetector::extractFeaturesFromFeatureRegion(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr& region_points,
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

  int n_edges_picked = 0;

  int point_info_count = 0;
  for (int i = cloud_curvatures_sorted.size() - 1; i >= 0; i--) {
    int ind = cloud_curvatures_sorted[i].first;
    if (!(*point_picked)[ind]) {
      if (cloud_curvatures_sorted[i].second <= 0.1) {
        break;
      }

      n_edges_picked++;
      (*point_picked)[ind] = true;

      if (n_edges_picked <= FLAGS_regbox_loam_edges_per_region) {
        edges->push_back(region_points->points[ind]);
        point_info_count++;
        markCurvatureRegionAsPicked(
            ind, sqrt(0.05), region_points, point_picked);
      } else {
        break;
      }
    }
  }

  int n_surfaces_picked = 0;

  for (int i = 0; i <= cloud_curvatures_sorted.size() - 1u; i++) {
    if (cloud_curvatures_sorted[i].second > 0.1)
      break;
    int ind = cloud_curvatures_sorted[i].first;
    if (n_surfaces_picked <= FLAGS_regbox_loam_surfaces_per_region) {
      if (!(*point_picked)[ind]) {
        surfaces->push_back(region_points->points[ind]);
        n_surfaces_picked++;
        markCurvatureRegionAsPicked(
            ind, sqrt(0.05), region_points, point_picked);
      }
    }
  }
}

void LoamFeatureDetector::markUnstablePointsAsPicked(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr& scan_line,
    std::vector<bool>* point_picked) {
  for (size_t point_idx = FLAGS_regbox_loam_curvature_region;
       point_idx < scan_line->size() - FLAGS_regbox_loam_curvature_region;
       point_idx++) {
    const pcl::PointXYZI& point = scan_line->points[point_idx];
    const float point_distance_m = point.getVector3fMap().norm();
    if (point_distance_m < 0.8 || point_distance_m > 60) {
      markCurvatureRegionAsPicked(point_idx, point_picked);
      continue;
    }

    const pcl::PointXYZI& previous_point = scan_line->points[point_idx - 1];
    const pcl::PointXYZI& next_point = scan_line->points[point_idx + 1];

    const float point_to_next_point_distance_m =
        (next_point.getVector3fMap() - point.getVector3fMap()).norm();

    if (point_to_next_point_distance_m > sqrt(0.1)) {
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

    float point_to_previous_point_distance_m =
        (point.getVector3fMap() - previous_point.getVector3fMap()).norm();

    // this will reject a point if the difference in distance of point and its
    // neighbors is outside a bound i.e. rejecting very sharp ramp like points
    const float point_distance_threshold_m = sqrt(0.0002) * point_distance_m;
    if (point_to_next_point_distance_m > point_distance_threshold_m &&
        point_to_previous_point_distance_m > point_distance_threshold_m) {
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
    const pcl::PointCloud<pcl::PointXYZI>::Ptr& scan_line,
    std::vector<bool>* point_picked) {
  (*point_picked)[point_idx] = true;
  for (int neighbor_idx = 1; neighbor_idx <= FLAGS_regbox_loam_curvature_region;
       neighbor_idx++) {
    const float point_to_neighbor_distance_m =
        (scan_line->points[point_idx + neighbor_idx].getVector3fMap() -
         scan_line->points[point_idx + neighbor_idx - 1].getVector3fMap())
            .norm();
    if (point_to_neighbor_distance_m > distance_threshold_m) {
      break;
    }
    (*point_picked)[point_idx + neighbor_idx] = true;
  }

  for (int neighbor_idx = 1; neighbor_idx <= FLAGS_regbox_loam_curvature_region;
       neighbor_idx++) {
    const float point_to_neighbor_distance_m =
        (scan_line->points[point_idx - neighbor_idx].getVector3fMap() -
         scan_line->points[point_idx - neighbor_idx + 1].getVector3fMap())
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
  for (int neighbor_idx = 0; neighbor_idx <= FLAGS_regbox_loam_curvature_region;
       neighbor_idx++) {
    (*point_picked)[point_idx + neighbor_idx] = true;
  }
}

void LoamFeatureDetector::downSampleFeatures(
    pcl::PointCloud<pcl::PointXYZI>::Ptr edges,
    pcl::PointCloud<pcl::PointXYZI>::Ptr surfaces) {
  pcl::VoxelGrid<pcl::PointXYZI> edge_filter;
  edge_filter.setInputCloud(edges);
  edge_filter.setLeafSize(
      FLAGS_regbox_loam_edge_filter_size_m,
      FLAGS_regbox_loam_edge_filter_size_m,
      FLAGS_regbox_loam_edge_filter_size_m);
  edge_filter.filter(*edges);

  pcl::VoxelGrid<pcl::PointXYZI> surface_filter;
  surface_filter.setInputCloud(surfaces);
  surface_filter.setLeafSize(
      FLAGS_regbox_loam_surface_filter_size_m,
      FLAGS_regbox_loam_surface_filter_size_m,
      FLAGS_regbox_loam_surface_filter_size_m);
  surface_filter.filter(*surfaces);
}

void LoamFeatureDetector::calculateCurvatures(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr& scan_line,
    CurvaturePairs* curvatures) {
  for (int point_idx = FLAGS_regbox_loam_curvature_region;
       point_idx <
       scan_line->points.size() - FLAGS_regbox_loam_curvature_region;
       point_idx++) {
    Eigen::Vector3f merged_point =
        2. * FLAGS_regbox_loam_curvature_region *
        scan_line->points[point_idx].getVector3fMap();
    for (int neighbor_idx = 1;
         neighbor_idx <= FLAGS_regbox_loam_curvature_region; neighbor_idx++) {
      merged_point -=
          scan_line->points[point_idx + neighbor_idx].getVector3fMap();
      merged_point -=
          scan_line->points[point_idx - neighbor_idx].getVector3fMap();
    }
    const CurvaturePair curvature_pair(point_idx, merged_point.squaredNorm());
    curvatures->push_back(curvature_pair);
  }
}

}  // namespace regbox
