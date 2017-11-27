#ifndef VI_MAP_HELPERS_VI_MAP_GEOMETRY_H_
#define VI_MAP_HELPERS_VI_MAP_GEOMETRY_H_

#include <vector>

#include <Eigen/Dense>
#include <posegraph/unique-id.h>
#include <vi-map/unique-id.h>
#include <vi-map/vi-map.h>

namespace vi_map {
class Vertex;
}  // namespace vi_map

namespace vi_map_helpers {

class VIMapGeometry {
 public:
  explicit VIMapGeometry(const vi_map::VIMap& map);

  // Positions, transformations etc.
  pose::Transformation getVisualFrame_T_G_C(
      const vi_map::VisualFrameIdentifier& frame_id) const;

  // Uses all good landmarks to compute median scene depth. If no landmarks are
  // found it returns infinity.
  double getMedianSceneDepth(
      const vi_map::VisualFrameIdentifier& frame_id) const;

  int getNeighboursWithinRange(
      const pose_graph::VertexId& vertex_id, double range_m,
      pose_graph::VertexIdSet* neighbours) const;

  void get_p_G_I_CovarianceEigenValuesAndVectorsAscending(
      const vi_map::MissionId& mission_id, Eigen::Vector3d* eigenvalues,
      Eigen::Matrix3d* eigenvectors) const;

  // Bearing vector from root vertex to average vertex position.
  Eigen::Vector3d get_bv_G_root_average(
      const vi_map::MissionId& mission_id) const;

  // Diverse bounding boxes.
  template <typename ObjectIdType>
  void getBoundingBox(Eigen::Vector3d* p_G_min, Eigen::Vector3d* p_G_max) const;
  template <typename ObjectIdType>
  void getBoundingBox(
      const vi_map::MissionId& mission_id, Eigen::Vector3d* p_G_min,
      Eigen::Vector3d* p_G_max) const;
  template <typename ObjectIdContainerType>
  void getBoundingBox(
      const ObjectIdContainerType& object_ids, Eigen::Vector3d* p_G_min,
      Eigen::Vector3d* p_G_max) const;

 private:
  const vi_map::VIMap& map_;
};

}  // namespace vi_map_helpers

#include "vi-map-helpers/vi-map-geometry-inl.h"

#endif  // VI_MAP_HELPERS_VI_MAP_GEOMETRY_H_
