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
  pickedpoints_.clear();
  for (int i = 0; i < N_SCANS; i++) {
    scan_lines.push_back(pcl::PointCloud<pcl::PointXYZI>::Ptr(
        new pcl::PointCloud<pcl::PointXYZI>));
  }

  pcl::PointCloud<pcl::PointXYZI> loam_input;
  for (int idx = 0u; idx < point_cloud->size(); idx++) {
    const pcl::PointXYZI& point = (*point_cloud)[idx];
    CHECK_GT(N_SCANS, point.intensity);
    CHECK_GE(point.intensity, 0);
    const double planar_distance = sqrt(point.x * point.x + point.y * point.y);
    if (planar_distance < 0.001 || planar_distance > 57.)
      continue;
    if (point.getVector3fMap().norm() < 1.)
      continue;
    if (abs(point.z) < 0.001)
      continue;
    if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
        !std::isfinite(point.z)) {
      continue;
    }
    scan_lines[static_cast<int>(point.intensity)]->push_back(point);
    loam_input.push_back(point);
  }
  for (const pcl::PointCloud<pcl::PointXYZI>::Ptr& scan_line : scan_lines) {
    if (scan_line->size() >= 2 * FLAGS_regbox_loam_curvature_region + 1) {
      extractFeaturesFromScanLine(scan_line, edges, surfaces);
    }
  }

  const std::string kLoamFeatureFrame = "/os_lidar";
  const std::string kLoamInputPointCloudTopic = "/loam_input_points";
  sensor_msgs::PointCloud2 loam_input_points_msg;
  pcl::toROSMsg(loam_input, loam_input_points_msg);
  loam_input_points_msg.header.frame_id = kLoamFeatureFrame;
  visualization::RVizVisualizationSink::publish(
      kLoamInputPointCloudTopic, loam_input_points_msg);

  // const std::string kLoamFeatureFrame = "/os_lidar";
  pickedpoints_.header.frame_id = kLoamFeatureFrame;
  const std::string kLoamUnstablePointCloudTopic = "/loam_unstable_points";
  sensor_msgs::PointCloud2 loam_unstable_points_msg;
  pcl::toROSMsg(pickedpoints_, loam_unstable_points_msg);
  visualization::RVizVisualizationSink::publish(
      kLoamUnstablePointCloudTopic, loam_unstable_points_msg);

  pcl::PointCloud<pcl::PointXYZI> edge_points = *edges;
  edge_points.header.frame_id = kLoamFeatureFrame;
  sensor_msgs::PointCloud2 loam_edge_points_msg;
  pcl::toROSMsg(edge_points, loam_edge_points_msg);
  const std::string kLoamEdgePointCloudTopic = "/loam_edge_points";
  visualization::RVizVisualizationSink::publish(
      kLoamEdgePointCloudTopic, loam_edge_points_msg);

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
  for (size_t idx = 0u; idx < cloud_curvatures.size(); ++idx) {
    if (point_picked[cloud_curvatures[idx].first]) {
      pcl::PointXYZI point = (*scan_line)[cloud_curvatures[idx].first];
      point.intensity = cloud_curvatures[idx].second;
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
    const pcl::PointCloud<pcl::PointXYZI>::Ptr& scan_line,
    std::vector<bool>* point_picked) {
  for (int point_idx = FLAGS_regbox_loam_curvature_region;
       point_idx < scan_line->size() - FLAGS_regbox_loam_curvature_region;
       point_idx++) {
    const pcl::PointXYZI& point = (*scan_line)[point_idx];
    const pcl::PointXYZI& previous_point = (*scan_line)[point_idx - 1];
    float alpha = acos(
        point.getVector3fMap().dot(previous_point.getVector3fMap()) /
        (point.getVector3fMap().norm() *
         previous_point.getVector3fMap().norm()));
    if (abs(alpha) > 1. * M_PI / 180.) {
      markCurvatureRegionAsPicked(point_idx, point_picked);
    }
    const float point_distance_m = point.getVector3fMap().norm();

    const pcl::PointXYZI& next_point = (*scan_line)[point_idx + 1];

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
    const pcl::PointCloud<pcl::PointXYZI>::Ptr& scan_line,
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
