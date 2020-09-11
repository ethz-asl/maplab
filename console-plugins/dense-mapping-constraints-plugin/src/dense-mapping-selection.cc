#include "dense-mapping/dense-mapping-selection.h"

#include <gflags/gflags.h>
#include <glog/logging.h>

namespace dense_mapping {

SelectionConfig SelectionConfig::fromGflags() {
  SelectionConfig config;
  config.recompute_all_constraints =
      FLAGS_dm_candidate_selection_recompute_all_constraints;
  config.recompute_invalid_constraints =
      FLAGS_dm_candidate_selection_recompute_invalid_constraints;

  config.constraint_min_switch_variable_value =
      FLAGS_dm_candidate_selection_min_switch_variable_value;

  return config;
}

bool hasGoodLoopClosureEdgeFromAToB(
    const SelectionConfig& config, const vi_map::VIMap& map,
    const pose_graph::VertexId& vertex_id_A,
    const pose_graph::VertexId& vertex_id_B,
    pose_graph::EdgeIdSet* constraints_to_delete_edge_ids) {
  CHECK_NOTNULL(constraints_to_delete_edge_ids);

  pose_graph::EdgeIdList outgoing_lc_edges_A;
  map.getOutgoingOfType(
      pose_graph::Edge::EdgeType::kLoopClosure, vertex_id_A,
      &outgoing_lc_edges_A);

  bool has_good_edge = false;

  for (const pose_graph::EdgeId& edge_id : outgoing_lc_edges_A) {
    CHECK(map.hasEdge(edge_id));
    const vi_map::LoopClosureEdge& edge =
        map.getEdgeAs<vi_map::LoopClosureEdge>(edge_id);
    CHECK_EQ(edge.from(), vertex_id_A);

    if (edge.to() != vertex_id_B) {
      continue;
    }

    const bool is_good_edge =
        edge.getSwitchVariable() >= config.constraint_min_switch_variable_value;

    // If the edge is not good and we want to recompute bad ones, or if we want
    // to recompute them regardless, we add them to the set to be removed later.
    if ((!is_good_edge && config.recompute_invalid_constraints) ||
        config.recompute_all_constraints) {
      constraints_to_delete_edge_ids->insert(edge_id);
    }

    has_good_edge |= is_good_edge;
  }
  return has_good_edge;
}

bool selectAlignmentCandidatePairs(
    const SelectionConfig& config, vi_map::VIMap* map_ptr,
    AlignmentCandidatePairs* candidate_pairs_ptr) {
  CHECK_NOTNULL(candidate_pairs_ptr);
  CHECK_NOTNULL(map_ptr);

  const size_t num_candidates_before = candidate_pairs_ptr->size();
  size_t num_removed_edges = 0u;
  size_t num_good_prior_edges = 0u;

  VLOG(1) << "Selecting final candidates from " << num_candidates_before
          << " initial candidates.";

  pose_graph::EdgeIdSet constraints_to_delete_edge_ids;
  AlignmentCandidatePairs::iterator it = candidate_pairs_ptr->begin();
  while (it != candidate_pairs_ptr->end()) {
    const AlignmentCandidatePair& alignment = *it;
    if (!alignment.isValid()) {
      VLOG(3) << "Invalid AlignmentCandidatePair:\n" << alignment;
      it = candidate_pairs_ptr->erase(it);
      continue;
    }
    const pose_graph::VertexId& vertex_id_B =
        alignment.candidate_B.closest_vertex_id;
    const pose_graph::VertexId& vertex_id_A =
        alignment.candidate_A.closest_vertex_id;
    const bool has_good_lc_edge =
        hasGoodLoopClosureEdgeFromAToB(
            config, *map_ptr, vertex_id_A, vertex_id_B,
            &constraints_to_delete_edge_ids) ||
        hasGoodLoopClosureEdgeFromAToB(
            config, *map_ptr, vertex_id_B, vertex_id_A,
            &constraints_to_delete_edge_ids);

    // Delete candidate if we have a good constraint already and if we don't
    // want to recompute all constraints.
    if (has_good_lc_edge) {
      ++num_good_prior_edges;
      if (!config.recompute_all_constraints) {
        it = candidate_pairs_ptr->erase(it);
        continue;
      }
    }

    ++it;
  }

  if (config.recompute_all_constraints ||
      config.recompute_invalid_constraints) {
    for (const pose_graph::EdgeId& edge_id : constraints_to_delete_edge_ids) {
      ++num_removed_edges;
      map_ptr->removeEdge(edge_id);
    }
  }

  VLOG(1) << "Reduced candidate set from " << num_candidates_before << " to "
          << candidate_pairs_ptr->size() << " based on " << num_good_prior_edges
          << " good prior constraints and removed " << num_removed_edges
          << " bad prior constraints.";

  return true;
}

}  // namespace dense_mapping
