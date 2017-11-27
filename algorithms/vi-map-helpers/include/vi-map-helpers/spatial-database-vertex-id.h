#ifndef VI_MAP_HELPERS_SPATIAL_DATABASE_VERTEX_ID_H_
#define VI_MAP_HELPERS_SPATIAL_DATABASE_VERTEX_ID_H_

#include <Eigen/Core>
#include <posegraph/unique-id.h>
#include <vi-map/vi-map.h>

#include "vi-map-helpers/spatial-database.h"

namespace vi_map_helpers {

class SpatialDatabaseVertexId : public SpatialDatabase<pose_graph::VertexId> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  SpatialDatabaseVertexId(
      const vi_map::VIMap& map, const Eigen::Vector3i& grid_resolution);

  SpatialDatabaseVertexId(
      const vi_map::VIMap& map, const Eigen::Vector3d& grid_cell_size);

  void getVertexIdsInRadiusWithinYawAngle(
      const aslam::Transformation& T_G_query_pose, double search_radius_meters,
      double yaw_angle_to_the_left_and_right_radians,
      pose_graph::VertexIdSet* neighbors) const;
};

}  // namespace vi_map_helpers

#endif  // VI_MAP_HELPERS_SPATIAL_DATABASE_VERTEX_ID_H_
