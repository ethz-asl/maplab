#include "sparse-graph/partitioners/lidar-partitioner.h"

#include <Eigen/Dense>
#include <Eigen/Eigenvalues>
#include <Eigen/Geometry>
#include <landmark-triangulation/pose-interpolator.h>

#include <glog/logging.h>

namespace spg {

LidarPartitioner::LidarPartitioner(const vi_map::VIMap& map)
    : BasePartitioner(map),
      point_cloud_resource_type_(backend::ResourceType::kPointCloudXYZI) {
  initializeSensorMapping();
}

void LidarPartitioner::initializeSensorMapping() {
  vi_map::MissionIdList mission_ids;
  map_.getAllMissionIds(&mission_ids);

  if (mission_ids.empty()) {
    LOG(ERROR) << "There are no missions in the loaded map. Aborting.";
    return;
  }

  // Find the mapping between mission IDs and sensor IDs.
  for (const vi_map::MissionId& mission_id : mission_ids) {
    CHECK(mission_id.isValid());
    const vi_map::VIMission& mission = map_.getMission(mission_id);
    const SensorsToResourceMap* lidar_sensor_id_to_res_id_map =
        mission.getAllSensorResourceIdsOfType(point_cloud_resource_type_);
    if (lidar_sensor_id_to_res_id_map == nullptr) {
      LOG(ERROR) << "Map has no appropriate lidar sensor associated with.";
      continue;
    }
    // We assume the first point cloud sensor is the correct one.
    mission_to_lidar_sensor_map_.emplace(
        mission_id, lidar_sensor_id_to_res_id_map->begin()->first);
  }
}

RepresentativeNode LidarPartitioner::getRepresentativesForSubmap(
    const pose_graph::VertexIdList& vertices) {
  if (mission_to_lidar_sensor_map_.empty()) {
    LOG(ERROR)
        << "Initialization of the mission id to sensor map failed. Aborting.";
    return RepresentativeNode();
  }
  const std::size_t n_vertices = vertices.size();
  if (n_vertices == 0) {
    LOG(ERROR) << "Received an empty vertex list. Aborting.";
    return RepresentativeNode();
  }

  constexpr int64_t tolerance_ns = 1e8;  // 100ms
  const landmark_triangulation::PoseInterpolator pose_interpolator;
  for (std::size_t i = 0u; i < n_vertices; ++i) {
    const vi_map::Vertex& vertex = map_.getVertex(vertices[i]);
    const vi_map::MissionId& mission_id = vertex.getMissionId();
    const vi_map::VIMission& mission = map_.getMission(mission_id);
    const int64_t ts_vertex_ns =
        vertex.getVisualNFrame().getMinTimestampNanoseconds();

    // Retrieve the closest LiDAR scan.
    resources::PointCloud pc;
    int64_t ts_pc_ns = -1;
    map_.getClosestSensorResource(
        mission, point_cloud_resource_type_,
        mission_to_lidar_sensor_map_[mission_id], ts_vertex_ns, tolerance_ns,
        &pc, &ts_pc_ns);

    // Interpolate LiDAR pose.
    Eigen::Matrix<int64_t, 1, Eigen::Dynamic> timestamps_ns =
        Eigen::Matrix<int64_t, 1, 1>::Constant(ts_pc_ns);

    aslam::TransformationVector T_M_B_vector;
    pose_interpolator.getPosesAtTime(
        map_, mission_id, timestamps_ns, &T_M_B_vector);
    CHECK_EQ(static_cast<int>(T_M_B_vector.size()), timestamps_ns.cols());
  }

  // Return averaged transformation with the used vertices.
  // aslam::Transformation averaged_T(average_quaternion, average_position);
  // return RepresentativeNode(averaged_T, vertices);
  return RepresentativeNode();
}

}  // namespace spg
