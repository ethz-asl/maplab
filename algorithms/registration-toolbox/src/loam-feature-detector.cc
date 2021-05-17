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
DEFINE_double(
    regbox_loam_max_angle_between_valid_neighbors_deg, 1.,
    "If angle is larger than this, the neighborhood is marked unstable. "
    "Can be used to filter out artificial features e.g. if filtered points "
    "are missing.");

namespace regbox {

LoamFeatureDetector::LoamFeatureDetector()
    : max_angle_between_valid_neighbors_rad_(
          M_PI * FLAGS_regbox_loam_max_angle_between_valid_neighbors_deg /
          180.) {}

void LoamFeatureDetector::extractLoamFeaturesFromPointCloud(
    const resources::PointCloud& point_cloud,
    resources::PointCloud* feature_cloud, size_t* n_edges, size_t* n_surfaces) {
  CHECK(point_cloud.hasRings());
  CHECK_NOTNULL(feature_cloud);

  PclPointCloudPtr<pcl::PointXYZ> edges(new pcl::PointCloud<pcl::PointXYZ>);
  PclPointCloudPtr<pcl::PointXYZ> surfaces(new pcl::PointCloud<pcl::PointXYZ>);

  std::vector<int> indices;
  std::map<size_t, pcl::PointCloud<pcl::PointXYZ>::Ptr> scan_lines;
  pickedpoints_.clear();

  for (size_t idx = 0u; idx < point_cloud.size(); idx++) {
    CHECK_GE(point_cloud.rings[idx], 0);
    pcl::PointXYZ point;
    point.x = point_cloud.xyz[3u * idx + 0];
    point.y = point_cloud.xyz[3u * idx + 1];
    point.z = point_cloud.xyz[3u * idx + 2];
    const double distance = point.getVector3fMap().norm();

    if (distance < kDistanceEpsillon)
      continue;
    if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
        !std::isfinite(point.z)) {
      continue;
    }
    auto& scan_line = scan_lines[point_cloud.rings[idx]];
    if (!scan_line) {
      scan_line = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    }
    scan_line->push_back(point);
  }
  for (auto& scan_line : scan_lines) {
    if (scan_line.second->size() >=
        2 * FLAGS_regbox_loam_curvature_region + 1) {
      sortScanLineAzimuthal(scan_line.second);
      extractFeaturesFromScanLine(scan_line.second, edges, surfaces);
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
  if (n_edges) {
    *n_edges = edges->size();
  }
  if (n_surfaces) {
    *n_surfaces = surfaces->size();
  }
}

void LoamFeatureDetector::sortScanLineAzimuthal(
    pcl::PointCloud<pcl::PointXYZ>::Ptr& scan_line) {
  CHECK_NOTNULL(scan_line);
  std::vector<std::pair<pcl::PointXYZ, float>> azimuthal_pairs;
  for (auto point : *scan_line) {
    const float theta = atan2(point.y, point.x);
    azimuthal_pairs.push_back(std::make_pair(point, theta));
  }
  std::sort(
      azimuthal_pairs.begin(), azimuthal_pairs.end(),
      [](const std::pair<pcl::PointXYZ, float>& a,
         const std::pair<pcl::PointXYZ, float>& b) {
        return a.second < b.second;
      });
  for (size_t idx = 0u; idx < scan_line->size(); ++idx) {
    scan_line->points[idx] = azimuthal_pairs[idx].first;
  }
}

void LoamFeatureDetector::extractFeaturesFromScanLine(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& scan_line,
    pcl::PointCloud<pcl::PointXYZ>::Ptr edges,
    pcl::PointCloud<pcl::PointXYZ>::Ptr surfaces) {
  CHECK_NOTNULL(scan_line);
  CHECK_NOTNULL(edges);
  CHECK_NOTNULL(surfaces);
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
      if (cloud_curvatures[idx].second <=
          FLAGS_regbox_loam_surface_curvature_threshold) {
        break;
      }
      if (edges_extracted_per_region
              [FLAGS_regbox_loam_feature_regions * ind / scan_line->size()] <
          FLAGS_regbox_loam_edges_per_region) {
        point_picked[ind] = true;
        edges_extracted_per_region
            [FLAGS_regbox_loam_feature_regions * ind / scan_line->size()]++;
        edges->push_back((*scan_line)[ind]);
        markCurvatureRegionAsPicked(
            ind, sqrt(kMaxCurvatureRegionNeighborDistanceSquaredm2), scan_line,
            &point_picked);
      } else {
        break;
      }
    }
  }

  for (int idx = 0; idx < cloud_curvatures.size(); idx++) {
    int ind = cloud_curvatures[idx].first;
    if (!point_picked[ind]) {
      if (cloud_curvatures[idx].second >
          FLAGS_regbox_loam_surface_curvature_threshold) {
        break;
      }
      if (surfaces_extracted_per_region
              [FLAGS_regbox_loam_feature_regions * ind / scan_line->size()] <
          FLAGS_regbox_loam_surfaces_per_region) {
        surfaces->push_back((*scan_line)[ind]);
        surfaces_extracted_per_region
            [FLAGS_regbox_loam_feature_regions * ind / scan_line->size()]++;
        markCurvatureRegionAsPicked(
            ind, sqrt(kMaxCurvatureRegionNeighborDistanceSquaredm2), scan_line,
            &point_picked);
      }
    }
  }
}

void LoamFeatureDetector::markUnstablePointsAsPicked(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& scan_line,
    std::vector<bool>* point_picked) {
  CHECK_NOTNULL(scan_line);
  CHECK_NOTNULL(point_picked);
  for (int point_idx = FLAGS_regbox_loam_curvature_region;
       point_idx < scan_line->size() - FLAGS_regbox_loam_curvature_region;
       point_idx++) {
    const pcl::PointXYZ& point = (*scan_line)[point_idx];
    const pcl::PointXYZ& previous_point = (*scan_line)[point_idx - 1];
    const float alpha = acos(
        point.getVector3fMap().dot(previous_point.getVector3fMap()) /
        (point.getVector3fMap().norm() *
         previous_point.getVector3fMap().norm()));
    if (abs(alpha) > max_angle_between_valid_neighbors_rad_) {
      markCurvatureRegionAsPicked(point_idx, point_picked);
    }
    const float point_distance_m = point.getVector3fMap().norm();

    const pcl::PointXYZ& next_point = (*scan_line)[point_idx + 1];

    const float squared_point_to_next_point_distance_m2 =
        (point.getVector3fMap() - next_point.getVector3fMap()).squaredNorm();

    if (squared_point_to_next_point_distance_m2 >
        kSquaredPointToNextPointDistancem2) {
      const float next_point_distance_m = next_point.getVector3fMap().norm();

      if (point_distance_m > next_point_distance_m) {
        float weighted_distance = (next_point.getVector3fMap() -
                                   (next_point_distance_m / point_distance_m) *
                                       point.getVector3fMap())
                                      .norm() /
                                  next_point_distance_m;

        //  In this case there is a gap between the points because the point is
        //  occluded by the next point and we mark the points before it
        if (weighted_distance < kWeightedDistanceThreshold) {
          markFirstHalfCurvatureRegionAsPicked(point_idx, point_picked);
          continue;
        }
      } else {
        float weighted_distance = (point.getVector3fMap() -
                                   (point_distance_m / next_point_distance_m) *
                                       next_point.getVector3fMap())
                                      .norm() /
                                  point_distance_m;
        //  In this case there is a gap between the points because the point is
        //  occluded by the previous point and we mark the points after it
        if (weighted_distance < kWeightedDistanceThreshold) {
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
        kRampFactor * squared_point_distance_m2;
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
  CHECK_NOTNULL(point_picked);
  for (int neighbor_idx = -FLAGS_regbox_loam_curvature_region;
       neighbor_idx <= FLAGS_regbox_loam_curvature_region; neighbor_idx++) {
    (*point_picked)[point_idx + neighbor_idx] = true;
  }
}

void LoamFeatureDetector::markCurvatureRegionAsPicked(
    const int& point_idx, const double& distance_threshold_m,
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& scan_line,
    std::vector<bool>* point_picked) {
  CHECK_NOTNULL(scan_line);
  CHECK_NOTNULL(point_picked);
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
  CHECK_NOTNULL(point_picked);
  for (int neighbor_idx = 0; neighbor_idx <= FLAGS_regbox_loam_curvature_region;
       neighbor_idx++) {
    (*point_picked)[point_idx - neighbor_idx] = true;
  }
}

void LoamFeatureDetector::markSecondHalfCurvatureRegionAsPicked(
    const int& point_idx, std::vector<bool>* point_picked) {
  CHECK_NOTNULL(point_picked);
  (*point_picked)[point_idx] = true;
  for (int neighbor_idx = 1; neighbor_idx <= FLAGS_regbox_loam_curvature_region;
       neighbor_idx++) {
    (*point_picked)[point_idx + neighbor_idx] = true;
  }
}

void LoamFeatureDetector::downSampleFeatures(
    pcl::PointCloud<pcl::PointXYZ>::Ptr edges,
    pcl::PointCloud<pcl::PointXYZ>::Ptr surfaces) {
  CHECK_NOTNULL(edges);
  CHECK_NOTNULL(surfaces);
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
  CHECK_NOTNULL(scan_line);
  CHECK_NOTNULL(curvatures);
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
