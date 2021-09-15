#include "dense-mapping/dense-mapping-selection.h"

#include <algorithm>
#include <list>
#include <random>
#include <vector>

#include <gflags/gflags.h>
#include <glog/logging.h>

namespace dense_mapping {

SelectionConfig SelectionConfig::fromGflags() {
  SelectionConfig config;

  // LC edge quality filtering.
  config.recompute_all_constraints =
      FLAGS_dm_candidate_selection_recompute_all_constraints;
  config.recompute_invalid_constraints =
      FLAGS_dm_candidate_selection_recompute_invalid_constraints;
  config.constraint_min_switch_variable_value =
      FLAGS_dm_candidate_selection_min_switch_variable_value;

  // LC edge generic filtering.
  config.max_number_of_candidates =
      FLAGS_dm_candidate_selection_max_number_of_candidates;
  config.filter_strategy = FLAGS_dm_candidate_selection_filter_strategy;
  config.prioritize_recent_candidates =
      FLAGS_dm_candidate_selection_prioritize_recent_candidates;

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

static void filter_candidates_based_on_quality(
    const SelectionConfig& config, vi_map::VIMap* map_ptr,
    AlignmentCandidatePairs* candidate_pairs_ptr) {
  CHECK_NOTNULL(candidate_pairs_ptr);
  CHECK_NOTNULL(map_ptr);
  const size_t num_candidates_before = candidate_pairs_ptr->size();
  std::size_t num_removed_edges = 0u;
  std::size_t num_good_prior_edges = 0u;

  VLOG(1) << "Selecting candidates based on quality from "
          << num_candidates_before << " initial candidates.";

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
}

static void remove_consecutive_candidates_from_priority(
    const std::size_t n_reserved_candidates,
    std::vector<AlignmentCandidatePairs::iterator>* v) {
  CHECK_NOTNULL(v);
  auto it = v->begin();
  for (std::size_t i = 0u; i < n_reserved_candidates; ++i) {
    if (it == v->end()) {
      break;
    }
    if ((*it)->type == ConstraintType::consecutive) {
      std::rotate(it, it + 1, v->end());
    } else {
      ++it;
    }
  }
}

static void filter_candidates_randomly(
    const std::size_t n_remaining_candidates_to_keep,
    const std::size_t n_reserved_candidates,
    AlignmentCandidatePairs* candidate_pairs_ptr) {
  CHECK_NOTNULL(candidate_pairs_ptr);

  // Create a vector of candidate iterators.
  const std::size_t n_candidates = candidate_pairs_ptr->size();
  if (n_candidates < n_remaining_candidates_to_keep) {
    return;
  }
  std::vector<AlignmentCandidatePairs::iterator> v(n_candidates);
  std::iota(v.begin(), v.end(), candidate_pairs_ptr->begin());

  // Sorter for the iterators based on the newest timestamp.
  auto sorter = [](const AlignmentCandidatePairs::iterator& lhs,
                   const AlignmentCandidatePairs::iterator& rhs) {
    const int64_t newest_lhs_ts_ns = lhs->getNewestTimestamp();
    const int64_t newest_rhs_ts_ns = rhs->getNewestTimestamp();
    return newest_lhs_ts_ns > newest_rhs_ts_ns;
  };

  // Sort the iterators to access the most recent ones.
  // Shuffle the remaining iterators.
  std::sort(v.begin(), v.end(), sorter);
  if (FLAGS_dm_candidate_selection_prioritize_recent_proximity_candidates) {
    remove_consecutive_candidates_from_priority(n_reserved_candidates, &v);
  }
  std::shuffle(
      v.begin() + n_reserved_candidates, v.end(),
      std::mt19937{std::random_device{}()});

  // Delete the elements from the original candidate list.
  const std::size_t n_remaining_candidates =
      n_candidates - n_reserved_candidates;
  const std::size_t n_candidates_to_delete =
      n_remaining_candidates -
      std::min(n_remaining_candidates, n_remaining_candidates_to_keep);
  auto it = v.begin() + n_reserved_candidates;
  const auto it_end = it + n_candidates_to_delete;
  for (; it != it_end; ++it) {
    candidate_pairs_ptr->erase(*it);
  }
}

static void filter_candidates_based_on_strategy(
    const SelectionConfig& config, vi_map::VIMap* map_ptr,
    AlignmentCandidatePairs* candidate_pairs_ptr) {
  if (config.max_number_of_candidates == 0) {
    return;
  }
  CHECK_NOTNULL(candidate_pairs_ptr);
  CHECK_NOTNULL(map_ptr);

  // Compute the bounds of the number of candidates to retrieve and keep.
  const std::size_t n_reserved_candidates = static_cast<std::size_t>(
      static_cast<double>(config.max_number_of_candidates) *
      config.prioritize_recent_candidates);
  const std::size_t n_remaining_candidates_to_keep =
      config.max_number_of_candidates - n_reserved_candidates;

  const std::size_t n_candidates_before = candidate_pairs_ptr->size();
  if (config.filter_strategy == "random") {
    filter_candidates_randomly(
        n_remaining_candidates_to_keep, n_reserved_candidates,
        candidate_pairs_ptr);
  } else {
    LOG(ERROR) << "Unknown filter strategy " << config.filter_strategy;
  }

  VLOG(1) << "Reduced candidate set from " << n_candidates_before << " to "
          << candidate_pairs_ptr->size() << " based on the "
          << config.filter_strategy << " strategy.";
}

bool selectAlignmentCandidatePairs(
    const SelectionConfig& config, vi_map::VIMap* map_ptr,
    AlignmentCandidatePairs* candidate_pairs_ptr) {
  CHECK_NOTNULL(candidate_pairs_ptr);
  CHECK_NOTNULL(map_ptr);

  // First, filter candidates based on their current edge quality.
  filter_candidates_based_on_quality(config, map_ptr, candidate_pairs_ptr);

  // Next, filter the remaining candidates based on their priority.
  filter_candidates_based_on_strategy(config, map_ptr, candidate_pairs_ptr);

  return true;
}

}  // namespace dense_mapping
