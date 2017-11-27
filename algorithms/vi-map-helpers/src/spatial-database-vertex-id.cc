#include "vi-map-helpers/spatial-database-vertex-id.h"

#include <maplab-common/quaternion-math.h>

namespace vi_map_helpers {

SpatialDatabaseVertexId::SpatialDatabaseVertexId(
    const vi_map::VIMap& map, const Eigen::Vector3i& grid_resolution)
    : SpatialDatabase<pose_graph::VertexId>(map, grid_resolution) {}

SpatialDatabaseVertexId::SpatialDatabaseVertexId(
    const vi_map::VIMap& map, const Eigen::Vector3d& grid_cell_size)
    : SpatialDatabase<pose_graph::VertexId>(map, grid_cell_size) {}

void SpatialDatabaseVertexId::getVertexIdsInRadiusWithinYawAngle(
    const aslam::Transformation& T_G_query_pose, double search_radius_meters,
    double yaw_angle_to_the_left_and_right_radians,
    std::unordered_set<pose_graph::VertexId>* neighbors) const {
  CHECK_NOTNULL(neighbors)->clear();

  getObjectIdsInRadius(
      T_G_query_pose.getPosition(), search_radius_meters, neighbors);

  std::unordered_set<pose_graph::VertexId>::iterator vertex_id_iterator =
      neighbors->begin();

  const aslam::Transformation T_query_pose_G = T_G_query_pose.inverse();
  while (vertex_id_iterator != neighbors->end()) {
    const pose_graph::VertexId& nn_vertex_id = *vertex_id_iterator;
    const aslam::Transformation& T_G_NN =
        SpatialDatabase<pose_graph::VertexId>::map_.getVertex_T_G_I(
            nn_vertex_id);

    const double yaw_angle_difference_radians =
        common::getYawAngleDifferenceRadians(T_G_query_pose, T_G_NN);

    if (yaw_angle_difference_radians <=
        yaw_angle_to_the_left_and_right_radians) {
      ++vertex_id_iterator;
    } else {
      vertex_id_iterator = neighbors->erase(vertex_id_iterator);
    }
  }
}

}  // namespace vi_map_helpers
