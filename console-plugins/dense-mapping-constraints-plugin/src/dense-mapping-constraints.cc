#include "dense-mapping/dense-mapping-constraints.h"

#include <aslam/common/covariance-helpers.h>
#include <aslam/common/pose-types.h>
#include <gflags/gflags.h>
#include <glog/logging.h>

namespace dense_mapping {

ConstraintsConfig ConstraintsConfig::fromGflags() {
  ConstraintsConfig config;
  config.constraint_switch_variable_value =
      FLAGS_dm_constraint_switch_variable_value;
  config.constraint_switch_variable_sigma =
      FLAGS_dm_constraint_switch_variable_sigma;
  return config;
}

static void applyCandidateToMap(
    const ConstraintsConfig& config, const AlignmentCandidatePair& alignment,
    vi_map::VIMap* map_ptr) {
  CHECK_NOTNULL(map_ptr);
  // Check if there are prior loop closures between those vertices.
  // TODO(mfehr): This might be ok, but we need to check first if the loop
  // closures not simply start accumulating.
  const vi_map::Vertex& vertex_A =
      map_ptr->getVertex(alignment.candidate_A.closest_vertex_id);
  const vi_map::Vertex& vertex_B =
      map_ptr->getVertex(alignment.candidate_B.closest_vertex_id);
  pose_graph::EdgeIdSet outgoing_edges;
  vertex_A.getOutgoingEdges(&outgoing_edges);
  std::unordered_set<pose_graph::EdgeId> previous_lc_edge_ids;
  for (const pose_graph::EdgeId& edge_id : outgoing_edges) {
    const pose_graph::Edge& edge =
        map_ptr->getEdgeAs<pose_graph::Edge>(edge_id);
    if (edge.getType() == pose_graph::Edge::EdgeType::kLoopClosure) {
      CHECK_EQ(edge.from(), vertex_A.id());
      if (edge.to() == vertex_B.id()) {
        previous_lc_edge_ids.insert(edge.id());
      }
    }
  }
  if (!previous_lc_edge_ids.empty()) {
    LOG(WARNING) << "There are " << previous_lc_edge_ids.size()
                 << " prior loop closures from vertex " << vertex_A.id()
                 << " to vertex " << vertex_B.id() << "!";
  }

  // Transfer the alignment from a transformation between two resources in
  // sensor frame (and potentially different timestamp than the vertices), to
  // a vertex to vertex constraint in the base sensor frame. NOTE: Frame
  // convention: G: Global frame B: Sensor base frame, corresponds to the
  // timestamp of the vertex. S: Sensor frame, corresponds to the timestamp of
  // the resources. This means that T_SB_BB can different from simply the
  // calibration from sensor frame to base sensor frame, but also includes the
  // offset between vertex time and the resource time.
  const aslam::Transformation& T_G_B_vertex_A =
      alignment.candidate_A.T_G_B_closest_vertex;
  const aslam::Transformation& T_G_S_resource_A =
      alignment.candidate_A.T_G_S_resource;
  const aslam::Transformation T_SA_BA =
      T_G_S_resource_A.inverse() * T_G_B_vertex_A;

  const aslam::Transformation& T_G_B_vertex_B =
      alignment.candidate_B.T_G_B_closest_vertex;
  const aslam::Transformation& T_G_S_resource_B =
      alignment.candidate_B.T_G_S_resource;
  const aslam::Transformation T_SB_BB =
      T_G_S_resource_B.inverse() * T_G_B_vertex_B;

  // Transformation between the two closest vertices.
  const aslam::Transformation T_BB_BA =
      T_SB_BB.inverse() * alignment.T_SB_SA_final * T_SA_BA;

  // TODO(mfehr): Check if this makes sense.
  aslam::TransformationCovariance T_BB_BA_covariance;
  aslam::common::rotateCovariance(
      T_SA_BA.inverse(), alignment.T_SB_SA_final_covariance,
      &T_BB_BA_covariance);

  pose_graph::EdgeId loop_closure_edge_id;
  aslam::generateId(&loop_closure_edge_id);
  CHECK(loop_closure_edge_id.isValid());
  vi_map::Edge::UniquePtr loop_closure_edge(new vi_map::LoopClosureEdge(
      loop_closure_edge_id, alignment.candidate_B.closest_vertex_id,
      alignment.candidate_A.closest_vertex_id,
      config.constraint_switch_variable_value,
      config.constraint_switch_variable_sigma, T_BB_BA, T_BB_BA_covariance));

  // Insert LC edge into the pose graph.
  map_ptr->addEdge(std::move(loop_closure_edge));

  VLOG(3)
      << "Added dense mapping constraint as loop closure edge between vertex "
      << alignment.candidate_A.closest_vertex_id.hexString() << " and vertex "
      << alignment.candidate_B.closest_vertex_id.hexString() << '.';
}

bool applyConstraintsToMap(
    const ConstraintsConfig& config,
    const AlignmentCandidatePairs& candidate_pairs, vi_map::VIMap* map_ptr) {
  CHECK_NOTNULL(map_ptr);

  VLOG(1) << "Adding dense mapping constraints to map.";
  uint32_t num_constraints_added = 0u;

  for (const AlignmentCandidatePair& alignment : candidate_pairs) {
    if (!alignment.isValid()) {
      LOG(WARNING) << "Invalid AlignmentCandidatePair:\n" << alignment;
      continue;
    }

    if (!alignment.success) {
      VLOG(3) << "Failed Registration:\n" << alignment;
      continue;
    }
    try {
      applyCandidateToMap(config, alignment, map_ptr);
      ++num_constraints_added;
    } catch (const std::exception&) {
      VLOG(3) << "Adding dense mapping constraint failed:\n" << alignment;
    }
  }

  VLOG(1) << "Added " << num_constraints_added
          << " dense mapping constraints (LC edges) to the map.";
  return true;
}

}  // namespace dense_mapping
