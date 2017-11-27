#ifndef MAP_OPTIMIZATION_LEGACY_TEST_6DOF_VI_MAP_GEN_H_
#define MAP_OPTIMIZATION_LEGACY_TEST_6DOF_VI_MAP_GEN_H_

#include <vi-map/vi-map.h>

#include "map-optimization-legacy/test/6dof-pose-graph-gen.h"
#include "map-optimization-legacy/test/6dof-test-trajectory-gen.h"

namespace map_optimization_legacy {

class ViwlsGraph;

class SixDofVIMapGenerator {
 public:
  SixDofVIMapGenerator() {}
  virtual ~SixDofVIMapGenerator() {}

  void generateVIMap();

  vi_map::VIMap vi_map_;
  vi_map::PoseGraph& posegraph_ = vi_map_.posegraph;
  Eigen::Vector3d getTrueVertexPosition(
      const pose_graph::VertexId& vertex_id) const;
  Eigen::Vector4d getTrueVertexRotation(
      const pose_graph::VertexId& vertex_id) const;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  SixDofPoseGraphGenerator graph_gen_;
};

};  // namespace map_optimization_legacy

#endif  // MAP_OPTIMIZATION_LEGACY_TEST_6DOF_VI_MAP_GEN_H_
