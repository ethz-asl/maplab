#ifndef VI_MAP_6DOF_VI_MAP_GEN_H_
#define VI_MAP_6DOF_VI_MAP_GEN_H_

#include <vi-map/vi-map.h>

#include "vi-map/6dof-pose-graph-gen.h"
#include "vi-map/6dof-test-trajectory-gen.h"

namespace vi_map {

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

};  // namespace vi_map

#endif  // VI_MAP_6DOF_VI_MAP_GEN_H_
