#ifndef DENSE_MAPPING_DENSE_MAPPING_SEARCH_H_
#define DENSE_MAPPING_DENSE_MAPPING_SEARCH_H_

#include <vector>

#include <gflags/gflags.h>
#include <glog/logging.h>
#include <vi-map/vi-map.h>

#include "dense-mapping/dense-mapping-common.h"
#include "dense-mapping/dense-mapping-gflags.h"

namespace dense_mapping {

struct SearchConfig {
  static SearchConfig fromGflags();

  std::vector<backend::ResourceType> resource_types;

  bool enable_intra_mission_consecutive_search;
  double consecutive_search_max_delta_time_s;
  double consecutive_search_max_delta_position_m;
  double consecutive_search_max_delta_rotation_deg;

  bool enable_intra_mission_proximity_search;
  bool enable_inter_mission_proximity_search;
  double proximity_search_max_delta_position_m;
  double proximity_search_max_delta_rotation_deg;
  double proximity_search_min_distance_along_graph_m;
  uint32_t proximity_search_take_closest_n_candidates;

  bool enable_intra_mission_global_search;
  bool enable_inter_mission_global_search;
  bool enable_incremental_submap_search;
};

void createCandidatePair(
    const AlignmentCandidate& candidate_A,
    const AlignmentCandidate& candidate_B,
    AlignmentCandidatePair* candidate_pair_ptr);

void findAllAlignmentCandidates(
    const SearchConfig& config, const vi_map::VIMap& map,
    const vi_map::MissionIdList& mission_ids,
    MissionToAlignmentCandidatesMap* candidates_per_mission_ptr);

bool searchForAlignmentCandidatePairs(
    const SearchConfig& config, const vi_map::VIMap& map,
    const vi_map::MissionIdList& mission_ids,
    AlignmentCandidatePairs* candidate_pairs_ptr);

bool searchForConsecutiveAlignmentCandidatePairs(
    const SearchConfig& config, const vi_map::MissionIdList& mission_ids,
    const MissionToAlignmentCandidatesMap& candidates_per_mission,
    AlignmentCandidatePairs* candidate_pairs_ptr);

bool searchForProximityBasedAlignmentCandidatePairs(
    const SearchConfig& config, const vi_map::MissionIdList& mission_ids,
    const MissionToAlignmentCandidatesMap& candidates_per_mission,
    AlignmentCandidatePairs* candidate_pairs_ptr);

bool searchForProximityBasedAlignmentCandidatePairsBetweenTwoMissions(
    const SearchConfig& config, const vi_map::MissionId& mission_A,
    const vi_map::MissionId& mission_B,
    const AlignmentCandidateList& candidates_A,
    const PositionMatrix& G_p_G_S_matrix_A,
    const Eigen::Matrix<double, 1, Eigen::Dynamic>&
        distances_to_first_vertex_along_graph_m_A,
    const TimestampNsVector& timestamp_ns_vector_A,
    const AlignmentCandidateList& candidates_B,
    const PositionMatrix& G_p_G_S_matrix_B,
    const Eigen::Matrix<double, 1, Eigen::Dynamic>&
        distances_to_first_vertex_along_graph_m_B,
    const TimestampNsVector& timestamp_ns_vector_B,
    AlignmentCandidatePairs* candidate_pairs_ptr);

bool searchGloballyForAlignmentCandidatePairs(
    const SearchConfig& config, const vi_map::VIMap& map,
    const vi_map::MissionIdList& mission_ids,
    const MissionToAlignmentCandidatesMap& candidates_per_mission,
    AlignmentCandidatePairs* candidate_pairs_ptr);

bool searchGloballyForAlignmentCandidatePairsBetweenTwoMissions(
    const SearchConfig& config, const vi_map::VIMap& map,
    const vi_map::MissionId& mission_A, const vi_map::MissionId& mission_B,
    const AlignmentCandidateList& candidates_A,
    const AlignmentCandidateList& candidates_B,
    AlignmentCandidatePairs* candidate_pairs_ptr);

bool searchForIncrementalSubmapAlignmentCandidatePairs(
    const SearchConfig& config, const vi_map::MissionIdList& mission_ids,
    const MissionToAlignmentCandidatesMap& candidates_per_mission,
    AlignmentCandidatePairs* candidate_pairs_ptr);

}  // namespace dense_mapping

#endif  // DENSE_MAPPING_DENSE_MAPPING_SEARCH_H_
