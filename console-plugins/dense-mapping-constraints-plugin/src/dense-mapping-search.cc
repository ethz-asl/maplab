#include "dense-mapping/dense-mapping-search.h"

#include <unordered_map>
#include <utility>

#include <Eigen/Core>
#include <aslam/common/pose-types.h>
#include <gflags/gflags.h>
#include <glog/logging.h>
#include <landmark-triangulation/pose-interpolator.h>
#include <map-resources/temporal-resource-id-buffer.h>
#include <maplab-common/accessors.h>
#include <vi-map/vertex.h>
#include <vi-map/vi-map.h>

namespace dense_mapping {

SearchConfig SearchConfig::fromGflags() {
  SearchConfig config;

  CHECK(backend::csvStringToResourceTypeList(
      FLAGS_dm_resource_types, &config.resource_types))
      << "Invalid list of dense mapping resource types: '"
      << FLAGS_dm_resource_types << "'";
  for (const auto& resource_type : config.resource_types) {
    CHECK(kSupportedResourceTypes.count(resource_type) > 0u)
        << "Resource type '"
        << backend::ResourceTypeNames[static_cast<int>(resource_type)]
        << "' is not supported by dense mapping.";
  }

  config.enable_intra_mission_consecutive_search =
      FLAGS_dm_candidate_search_enable_intra_mission_consecutive;

  config.consecutive_search_max_delta_time_s =
      FLAGS_dm_candidate_search_consecutive_max_delta_time_s;
  config.consecutive_search_max_delta_position_m =
      FLAGS_dm_candidate_search_consecutive_max_delta_position_m;
  config.consecutive_search_max_delta_rotation_deg =
      FLAGS_dm_candidate_search_consecutive_max_delta_rotation_deg;

  config.enable_intra_mission_proximity_search =
      FLAGS_dm_candidate_search_enable_intra_mission_proximity;
  config.enable_inter_mission_proximity_search =
      FLAGS_dm_candidate_search_enable_inter_mission_proximity;

  config.proximity_search_max_delta_position_m =
      FLAGS_dm_candidate_search_proximity_max_delta_position_m;
  config.proximity_search_max_delta_rotation_deg =
      FLAGS_dm_candidate_search_proximity_max_delta_rotation_deg;
  config.proximity_search_min_distance_along_graph_m =
      FLAGS_dm_candidate_search_proximity_min_distance_along_graph_m;
  config.proximity_search_take_closest_n_candidates =
      FLAGS_dm_candidate_search_proximity_take_closest_n_candidates;

  config.enable_inter_mission_global_search =
      FLAGS_dm_candidate_search_enable_inter_mission_global;
  config.enable_intra_mission_global_search =
      FLAGS_dm_candidate_search_enable_intra_mission_global;

  return config;
}

bool searchForAlignmentCandidatePairs(
    const SearchConfig& config, const vi_map::VIMap& map,
    const vi_map::MissionIdList& mission_ids,
    AlignmentCandidatePairs* candidate_pairs_ptr) {
  CHECK_NOTNULL(candidate_pairs_ptr)->clear();

  MissionToAlignmentCandidatesMap candidates_per_mission;
  try {
    findAllAlignmentCandidates(
        config, map, mission_ids, &candidates_per_mission);
  } catch (std::exception& e) {
    LOG(ERROR) << "Finding alignment pairs failed. Aborting.";
    return false;
  }

  if (config.enable_intra_mission_consecutive_search) {
    if (!searchForConsecutiveAlignmentCandidatePairs(
            config, mission_ids, candidates_per_mission, candidate_pairs_ptr)) {
      return false;
    }
  }
  VLOG(1) << "Found " << candidate_pairs_ptr->size()
          << " additional candidate pairs with consecutive search.";

  const uint32_t num_pairs_before_proximity_search =
      candidate_pairs_ptr->size();
  if (config.enable_intra_mission_proximity_search ||
      config.enable_inter_mission_proximity_search) {
    if (!searchForProximityBasedAlignmentCandidatePairs(
            config, mission_ids, candidates_per_mission, candidate_pairs_ptr)) {
      return false;
    }
  }
  VLOG(1) << "Found "
          << (candidate_pairs_ptr->size() - num_pairs_before_proximity_search)
          << " additional candidate pairs with proximity search.";

  const uint32_t num_pairs_before_global_search = candidate_pairs_ptr->size();
  if (config.enable_intra_mission_global_search ||
      config.enable_inter_mission_global_search) {
    if (!searchGloballyForAlignmentCandidatePairs(
            config, map, mission_ids, candidates_per_mission,
            candidate_pairs_ptr)) {
      return false;
    }
  }
  VLOG(1) << "Found "
          << (candidate_pairs_ptr->size() - num_pairs_before_global_search)
          << " additional candidate pairs with global search.";

  return true;
}

void findAllAlignmentCandidates(
    const SearchConfig& config, const vi_map::VIMap& map,
    const vi_map::MissionIdList& mission_ids,
    MissionToAlignmentCandidatesMap* candidates_per_mission_ptr) {
  CHECK_NOTNULL(candidates_per_mission_ptr)->clear();

  const vi_map::SensorManager& sensor_manager = map.getSensorManager();

  for (const vi_map::MissionId& mission_id : mission_ids) {
    AlignmentCandidateList& candidates =
        (*candidates_per_mission_ptr)[mission_id];

    const vi_map::VIMission& mission = map.getMission(mission_id);
    const aslam::Transformation& T_G_M =
        map.getMissionBaseFrameForMission(mission_id).get_T_G_M();

    // Check if there is IMU data to interpolate the optional sensor poses.
    landmark_triangulation::VertexToTimeStampMap vertex_to_time_map;
    int64_t min_timestamp_ns;
    int64_t max_timestamp_ns;
    const landmark_triangulation::PoseInterpolator pose_interpolator;
    pose_interpolator.getVertexToTimeStampMap(
        map, mission_id, &vertex_to_time_map, &min_timestamp_ns,
        &max_timestamp_ns);
    if (vertex_to_time_map.empty()) {
      LOG(WARNING) << "Couldn't find any IMU data to interpolate exact "
                   << "sensor position in mission " << mission_id;
      continue;
    }

    // Get timestamps of all vertices.
    common::TemporalBuffer<pose_graph::VertexId> temporal_vertex_buffer;
    {
      pose_graph::VertexIdList vertex_ids;
      map.getAllVertexIdsInMissionAlongGraph(mission_id, &vertex_ids);

      for (const pose_graph::VertexId& vertex_id : vertex_ids) {
        const int64_t vertex_timestamp_ns =
            map.getVertex(vertex_id).getMinTimestampNanoseconds();
        temporal_vertex_buffer.addValue(vertex_timestamp_ns, vertex_id);
      }
    }

    // Iterate over desired resource types.
    for (const backend::ResourceType& resource_type : config.resource_types) {
      // Get all sensor resources of this supported type.
      const auto* resources_of_type_ptr =
          mission.getAllSensorResourceIdsOfType(resource_type);
      if (resources_of_type_ptr == nullptr) {
        LOG(WARNING)
            << "No resources of type '"
            << backend::ResourceTypeNames[static_cast<int>(resource_type)]
            << "(" << static_cast<int>(resource_type) << ")' found in mission "
            << mission_id;
        continue;
      }

      // Iterate over all sensors that have this resource type.
      for (const auto& sensor_to_resource_buffer_map : *resources_of_type_ptr) {
        const aslam::SensorId& sensor_id = sensor_to_resource_buffer_map.first;
        const backend::TemporalResourceIdBuffer& resource_buffer =
            sensor_to_resource_buffer_map.second;

        if (resource_buffer.empty()) {
          continue;
        }

        int64_t oldest_timestamp_ns, newest_timestamp_ns;
        resource_buffer.getOldestTime(&oldest_timestamp_ns);
        resource_buffer.getNewestTime(&newest_timestamp_ns);
        VLOG(1) << "Extracting candidates from resource type '"
                << backend::ResourceTypeNames[static_cast<int>(resource_type)]
                << "' with time range ["
                << aslam::time::timeNanosecondsToString(oldest_timestamp_ns)
                << ", "
                << aslam::time::timeNanosecondsToString(newest_timestamp_ns)
                << "].";

        // Get transformation between reference (e.g. IMU) and sensor.
        const aslam::Transformation& T_B_S =
            sensor_manager.getSensor_T_B_S(sensor_id);

        // Get a mapping from resource to closest vertex.
        std::map<int64_t, pose_graph::VertexId, std::less<int64_t>>
            resource_timestamp_to_closest_vertex_id_map;
        for (const auto& stamped_resource : resource_buffer) {
          const int64_t timestamp_resource_ns = stamped_resource.first;

          if (timestamp_resource_ns < min_timestamp_ns ||
              timestamp_resource_ns > max_timestamp_ns) {
            VLOG(3)
                << "The map contains a resource of type '"
                << backend::ResourceTypeNames[static_cast<int>(resource_type)]
                << "(" << static_cast<int>(resource_type) << ")' at "
                << aslam::time::timeNanosecondsToString(min_timestamp_ns)
                << " that is not within the time range of the pose-graph ["
                << aslam::time::timeNanosecondsToString(min_timestamp_ns)
                << ", "
                << aslam::time::timeNanosecondsToString(max_timestamp_ns)
                << "]";
            continue;
          }

          pose_graph::VertexId closest_vertex_id;
          if (temporal_vertex_buffer.getNearestValueToTime(
                  timestamp_resource_ns, &closest_vertex_id)) {
            CHECK(closest_vertex_id.isValid());
            resource_timestamp_to_closest_vertex_id_map[timestamp_resource_ns] =
                closest_vertex_id;
          }
        }
        const size_t num_used_resources =
            resource_timestamp_to_closest_vertex_id_map.size();

        // Interpolate all resource poses.
        // Collect all timestamps that need to be interpolated.
        Eigen::Matrix<int64_t, 1, Eigen::Dynamic> resource_timestamps(
            num_used_resources);
        size_t idx = 0u;
        for (const auto& resource_timestamp_with_vertex_id :
             resource_timestamp_to_closest_vertex_id_map) {
          const int64_t timestamp_resource_ns =
              resource_timestamp_with_vertex_id.first;
          CHECK_GE(timestamp_resource_ns, min_timestamp_ns);
          CHECK_LE(timestamp_resource_ns, max_timestamp_ns);
          CHECK(resource_timestamp_with_vertex_id.second.isValid());

          resource_timestamps[idx] = timestamp_resource_ns;
          ++idx;
        }

        // Interpolate poses for every resource.
        aslam::TransformationVector T_M_B_vector;
        pose_interpolator.getPosesAtTime(
            map, mission_id, resource_timestamps, &T_M_B_vector);
        CHECK_EQ(
            static_cast<int>(T_M_B_vector.size()), resource_timestamps.size());
        CHECK_EQ(T_M_B_vector.size(), num_used_resources);

        // Iterate over each timestamp/resource id pair.
        idx = 0u;
        for (const auto& resource_timestamp_with_vertex_id :
             resource_timestamp_to_closest_vertex_id_map) {
          const int64_t timestamp_ns = resource_timestamp_with_vertex_id.first;

          const aslam::Transformation& T_M_B_resource = T_M_B_vector[idx];
          const aslam::Transformation T_G_S_resource =
              T_G_M * T_M_B_resource * T_B_S;

          AlignmentCandidate candidate;
          candidate.mission_id = mission_id;
          candidate.sensor_id = sensor_id;
          candidate.timestamp_ns = timestamp_ns;
          candidate.resource_type = resource_type;
          candidate.T_G_S_resource = T_G_S_resource;

          candidate.closest_vertex_id =
              resource_timestamp_with_vertex_id.second;
          candidate.T_G_B_closest_vertex =
              T_G_M * map.getVertex(candidate.closest_vertex_id).get_T_M_I();

          CHECK(candidate.isValid());
          candidates.emplace_back(std::move(candidate));

          ++idx;
        }
      }
    }
    VLOG(1) << "Found " << candidates.size() << " candidates in mission "
            << mission_id << ".";
  }
}

bool candidatesAreTemporallyTooFar(
    const SearchConfig& config, const AlignmentCandidate& candidate_A,
    const AlignmentCandidate& candidate_B) {
  // Compute delta time.
  const int64_t delta_time_ns =
      std::abs(candidate_B.timestamp_ns - candidate_A.timestamp_ns);
  CHECK_GT(delta_time_ns, 0);  // The candidates should be strictly increasing.

  const bool too_late = aslam::time::nanoSecondsToSeconds(delta_time_ns) >
                        config.consecutive_search_max_delta_time_s;
  VLOG_IF(5, too_late) << "Candidates have too large delta time: "
                       << aslam::time::timeNanosecondsToString(delta_time_ns);
  return too_late;
}

bool candidatesAreSpatiallyTooFar(
    const SearchConfig& config, const AlignmentCandidate& candidate_A,
    const AlignmentCandidate& candidate_B) {
  static constexpr double kRadToDeg = 180.0 / M_PI;
  const aslam::Transformation T_B_A =
      candidate_B.T_G_S_resource.inverse() * candidate_A.T_G_S_resource;

  const double sensor_delta_position_m = T_B_A.getPosition().norm();
  const double sensor_delta_rotation_deg =
      aslam::AngleAxis(T_B_A.getRotation()).angle() * kRadToDeg;

  const bool delta_position_too_large =
      sensor_delta_position_m > config.consecutive_search_max_delta_position_m;

  const bool delta_angle_too_large =
      std::abs(sensor_delta_rotation_deg) >
      config.consecutive_search_max_delta_rotation_deg;

  VLOG_IF(5, delta_position_too_large)
      << "Candidates have too large delta position: " << sensor_delta_position_m
      << "m";
  VLOG_IF(5, delta_angle_too_large)
      << "Candidates have too large delta angle: "
      << std::abs(sensor_delta_rotation_deg) << "deg";

  return delta_position_too_large || delta_angle_too_large;
}

void createCandidatePair(
    const AlignmentCandidate& candidate_A,
    const AlignmentCandidate& candidate_B,
    AlignmentCandidatePair* candidate_pair_ptr) {
  CHECK_NOTNULL(candidate_pair_ptr);
  candidate_pair_ptr->candidate_A = candidate_A;
  candidate_pair_ptr->candidate_B = candidate_B;
  // Use relative pose of pose graph as initial guess.
  candidate_pair_ptr->T_SB_SA_init =
      candidate_B.T_G_S_resource.inverse() * candidate_A.T_G_S_resource;
}

void addCandidatePair(
    const AlignmentCandidate& candidate_A,
    const AlignmentCandidate& candidate_B,
    AlignmentCandidatePairs* candidate_pairs_ptr) {
  CHECK_NOTNULL(candidate_pairs_ptr);
  AlignmentCandidatePair pair;
  createCandidatePair(candidate_A, candidate_B, &pair);
  candidate_pairs_ptr->emplace(pair);
}

bool searchForConsecutiveAlignmentCandidatePairs(
    const SearchConfig& config, const vi_map::MissionIdList& mission_ids,
    const MissionToAlignmentCandidatesMap& candidates_per_mission,
    AlignmentCandidatePairs* candidate_pairs_ptr) {
  CHECK_NOTNULL(candidate_pairs_ptr);

  for (const vi_map::MissionId& mission_id : mission_ids) {
    if (candidates_per_mission.count(mission_id) == 0u) {
      continue;
    }

    VLOG(1) << "Searching for consecutive candidates in mission " << mission_id;

    const AlignmentCandidateList& candidates =
        candidates_per_mission.at(mission_id);

    const size_t num_candidates = candidates.size();
    if (num_candidates < 2u) {
      return true;
    }

    const AlignmentCandidate* candidate_A = nullptr;
    const AlignmentCandidate* candidate_B = nullptr;

    size_t current_idx = 0u;
    while (current_idx < num_candidates) {
      // If we have not set a first candidate yet, we need to do now and move
      // on.
      if (candidate_A == nullptr) {
        CHECK_LT(current_idx, num_candidates);
        candidate_A = &(candidates[current_idx]);
        candidate_B = nullptr;
        VLOG(5) << aslam::time::timeNanosecondsToString(
                       candidate_A->timestamp_ns)
                << " - none (init)";

        ++current_idx;
        continue;
      }

      // Fetch current candidate for B.
      CHECK_LT(current_idx, num_candidates);
      const AlignmentCandidate* current_candidate = &(candidates[current_idx]);

      // Now we should have two candidates to compare.
      CHECK_NOTNULL(current_candidate);
      CHECK_NOTNULL(candidate_A);

      VLOG(5) << aslam::time::timeNanosecondsToString(candidate_A->timestamp_ns)
              << " - "
              << aslam::time::timeNanosecondsToString(
                     current_candidate->timestamp_ns);

      // For consecutive constraints we don't want to mix sensors, since we
      // expect the candidates to be in temporal order for each sensor. The
      // candidate list however will include all sensor data for a single
      // sensor/type in temporal order, but not across sensor/type.
      if (candidate_A->sensor_id != current_candidate->sensor_id ||
          candidate_A->resource_type != current_candidate->resource_type) {
        candidate_A = current_candidate;
        candidate_B = nullptr;

        ++current_idx;
        continue;
      }
      CHECK_LT(candidate_A->timestamp_ns, current_candidate->timestamp_ns);

      // If the current B candidate tripped any of the criteria, we need to take
      // the previous B candidate, unless we don't have one, in which case we
      // move on.
      if (candidatesAreTemporallyTooFar(
              config, *candidate_A, *current_candidate) ||
          candidatesAreSpatiallyTooFar(
              config, *candidate_A, *current_candidate)) {
        if (candidate_B == nullptr) {
          LOG(WARNING) << "Dense mapping consecutive candidate search config "
                       << "is too strict or there is not enough data, could "
                       << "not find corresponding candidate for map at "
                       << candidate_A->timestamp_ns << "ns. Skipping one "
                       << "dense (sub) map.";

          // If we haven't found a candidate B yet, then we likely cannot find a
          // match for candidate A. This means the search config is too strict
          // or there is not enough data! Move on to try to find a new pair,
          // with the current candidate as a basse.
          candidate_A = current_candidate;
          candidate_B = nullptr;
          CHECK_NOTNULL(candidate_A);

          ++current_idx;
          continue;
        }

        // If we alread had a candidate B, we take the two candidates and
        // create a candidate pair and move on.
        addCandidatePair(*candidate_A, *candidate_B, candidate_pairs_ptr);
        candidate_A = candidate_B;
        candidate_B = nullptr;
        CHECK_NOTNULL(candidate_A);

        // NOTE: Do not move the index, since we want to try candidate B against
        // the current candidate in the next iteration.
        continue;
      } else {
        // Move candidate B one further, since we can skip the previous B
        // candidate and still fullfill all criteria.
        candidate_B = current_candidate;
        CHECK_NOTNULL(candidate_A);
        CHECK_NOTNULL(candidate_B);

        ++current_idx;
        continue;
      }
    }
  }
  return true;
}

// Extract position and timestamp as eigen matrices to allow for efficient
// nearest neighbor search. It also computes the distance along the
// graph to the first vertex for each candidate.
void extractPositionAndTimestampMatricesFromCandidateList(
    const AlignmentCandidateList& candidates,
    PositionMatrix* G_p_G_S_matrix_ptr,
    Eigen::Matrix<double, 1, Eigen::Dynamic>*
        distances_to_first_vertex_along_graph_m_ptr,
    TimestampNsVector* timestamp_ns_vector_ptr) {
  const size_t num_candidates = candidates.size();
  CHECK_GT(num_candidates, 0u);
  *CHECK_NOTNULL(G_p_G_S_matrix_ptr) = PositionMatrix(3, num_candidates);
  *CHECK_NOTNULL(timestamp_ns_vector_ptr) =
      TimestampNsVector(1, num_candidates);
  *CHECK_NOTNULL(distances_to_first_vertex_along_graph_m_ptr) =
      Eigen::Matrix<double, 1, Eigen::Dynamic>(1, num_candidates);

  const AlignmentCandidate& first_candidate = candidates[0];
  G_p_G_S_matrix_ptr->col(0) = first_candidate.T_G_S_resource.getPosition();
  (*timestamp_ns_vector_ptr)(0) = first_candidate.timestamp_ns;
  (*distances_to_first_vertex_along_graph_m_ptr)(0) = 0.0;

  for (size_t idx = 1u; idx < num_candidates; ++idx) {
    const AlignmentCandidate& candidate = candidates[idx];
    const auto& G_p_G_S = candidate.T_G_S_resource.getPosition();
    G_p_G_S_matrix_ptr->col(idx) = G_p_G_S;
    (*distances_to_first_vertex_along_graph_m_ptr)(idx) =
        (*distances_to_first_vertex_along_graph_m_ptr)(idx - 1) +
        (G_p_G_S - G_p_G_S_matrix_ptr->col(idx - 1)).norm();
    (*timestamp_ns_vector_ptr)(idx) = candidate.timestamp_ns;
  }
}

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
    AlignmentCandidatePairs* candidate_pairs_ptr) {
  CHECK_NOTNULL(candidate_pairs_ptr);

  VLOG(1) << "Searching for proximity candidates between missions " << mission_A
          << " and " << mission_B;

  const size_t num_candidates_A = candidates_A.size();
  CHECK_EQ(static_cast<int>(num_candidates_A), G_p_G_S_matrix_A.cols());
  CHECK_EQ(
      static_cast<int>(num_candidates_A),
      distances_to_first_vertex_along_graph_m_A.cols());
  CHECK_EQ(static_cast<int>(num_candidates_A), timestamp_ns_vector_A.cols());

  const size_t num_candidates_B = candidates_B.size();
  CHECK_EQ(static_cast<int>(num_candidates_B), G_p_G_S_matrix_B.cols());
  CHECK_EQ(
      static_cast<int>(num_candidates_B),
      distances_to_first_vertex_along_graph_m_B.cols());
  CHECK_EQ(static_cast<int>(num_candidates_B), timestamp_ns_vector_B.cols());

  const double distance_along_graph_threshold_m =
      config.proximity_search_min_distance_along_graph_m;
  const double max_delta_position_m_squared =
      config.proximity_search_max_delta_position_m *
      config.proximity_search_max_delta_position_m;

  for (size_t idx_A = 0u; idx_A < num_candidates_A; ++idx_A) {
    const auto squared_distances =
        (G_p_G_S_matrix_B.colwise() - G_p_G_S_matrix_A.col(idx_A))
            .colwise()
            .squaredNorm();
    const auto is_within_range =
        squared_distances.array() < max_delta_position_m_squared;

    const AlignmentCandidate& candidate_A = candidates_A[idx_A];

    AlignedMap<double, AlignmentCandidatePair>
        squared_distance_to_candidate_pair_map;
    for (size_t idx_B = 0u; idx_B < num_candidates_B; ++idx_B) {
      CHECK_LT(static_cast<int>(idx_B), squared_distances.cols());
      if (is_within_range(idx_B) == 0) {
        CHECK_GE(squared_distances(idx_B), max_delta_position_m_squared);
        continue;
      }
      CHECK_LT(squared_distances(idx_B), max_delta_position_m_squared);

      // For intra mission proximity search, we also need to check that the
      // distance along the graph is larger than a threshold, otherwise we
      // generate overlapping candidates with the consecutive search and a very
      // large number of candidates in general.
      if (mission_A == mission_B) {
        // If it is the same candidate, skip this pair.
        if (idx_B == idx_A) {
          continue;
        }

        const bool candidates_are_too_close_along_the_graph_m =
            std::abs(
                distances_to_first_vertex_along_graph_m_B(idx_B) -
                distances_to_first_vertex_along_graph_m_B(idx_A)) <
            distance_along_graph_threshold_m;
        if (candidates_are_too_close_along_the_graph_m) {
          continue;
        }
      }

      // Check rotation.
      const AlignmentCandidate& candidate_B = candidates_B[idx_B];
      const aslam::Transformation T_B_A =
          candidate_B.T_G_S_resource.inverse() * candidate_A.T_G_S_resource;
      if (std::abs(aslam::AngleAxis(T_B_A.getRotation()).angle() * kRadToDeg) >
          config.proximity_search_max_delta_rotation_deg) {
        continue;
      }

      // Create candidate pair and save it ordered by distance;
      AlignmentCandidatePair pair;
      createCandidatePair(candidate_A, candidate_B, &pair);
      squared_distance_to_candidate_pair_map.emplace(
          squared_distances(idx_B), std::move(pair));
    }

    if (VLOG_IS_ON(3) && !squared_distance_to_candidate_pair_map.empty()) {
      std::stringstream ss;
      ss << "\nFound " << squared_distance_to_candidate_pair_map.size()
         << " proximity candidates for this candidate at:";
      for (const auto& pair_w_distance :
           squared_distance_to_candidate_pair_map) {
        ss << "\n - " << std::sqrt(pair_w_distance.first) << "m";
      }
      VLOG(3) << ss.str();
    }

    // If enabled (param proximity_search_take_closest_n_candidates is greater
    // than 0), take the closest n candidates.
    uint32_t selected_pairs = 0u;
    for (const auto& pair_w_distance : squared_distance_to_candidate_pair_map) {
      if (config.proximity_search_take_closest_n_candidates > 0 &&
          selected_pairs >= config.proximity_search_take_closest_n_candidates) {
        break;
      }
      candidate_pairs_ptr->emplace(std::move(pair_w_distance.second));
      ++selected_pairs;
    }
  }
  return true;
}

bool searchForProximityBasedAlignmentCandidatePairs(
    const SearchConfig& config, const vi_map::MissionIdList& mission_ids,
    const MissionToAlignmentCandidatesMap& candidates_per_mission,
    AlignmentCandidatePairs* candidate_pairs_ptr) {
  CHECK_NOTNULL(candidate_pairs_ptr);

  vi_map::MissionIdList::const_iterator it_A = mission_ids.cbegin();
  for (; it_A != mission_ids.end(); ++it_A) {
    const vi_map::MissionId& mission_id_A = *it_A;
    const AlignmentCandidateList& candidates_A =
        candidates_per_mission.at(mission_id_A);

    if (candidates_A.empty()) {
      continue;
    }

    PositionMatrix G_p_G_S_matrix_A;
    Eigen::Matrix<double, 1, Eigen::Dynamic>
        distances_to_first_vertex_along_graph_m_A;
    TimestampNsVector timestamp_ns_vector_A;
    extractPositionAndTimestampMatricesFromCandidateList(
        candidates_A, &G_p_G_S_matrix_A,
        &distances_to_first_vertex_along_graph_m_A, &timestamp_ns_vector_A);

    vi_map::MissionIdList::const_iterator start_it_B = it_A;
    vi_map::MissionIdList::const_iterator end_it_B = mission_ids.cend();
    // If no intra-mission candidates are desired, advance the start iterator to
    // skip the current, same mission.
    if (!config.enable_intra_mission_proximity_search) {
      ++start_it_B;
    }
    // If no inter-mission candidates are desired, set end iterator to the one
    // after the current, same mission.
    if (!config.enable_inter_mission_proximity_search) {
      end_it_B = it_A + 1;
    }

    vi_map::MissionIdList::const_iterator it_B = start_it_B;
    for (; it_B != end_it_B; ++it_B) {
      const vi_map::MissionId& mission_id_B = *it_B;
      const AlignmentCandidateList& candidates_B =
          candidates_per_mission.at(mission_id_B);

      if (candidates_B.empty()) {
        continue;
      }

      // If we are looking within a mission, there is no need to extract the
      // position matrix and timestamp vector again.
      if (it_B == it_A) {
        CHECK_EQ(mission_id_B, mission_id_A);
        CHECK_EQ(candidates_B.size(), candidates_A.size());
        if (!searchForProximityBasedAlignmentCandidatePairsBetweenTwoMissions(
                config, mission_id_A, mission_id_B, candidates_A,
                G_p_G_S_matrix_A, distances_to_first_vertex_along_graph_m_A,
                timestamp_ns_vector_A, candidates_B, G_p_G_S_matrix_A,
                distances_to_first_vertex_along_graph_m_A,
                timestamp_ns_vector_A, candidate_pairs_ptr)) {
          return false;
        }
      } else {
        PositionMatrix G_p_G_S_matrix_B;
        Eigen::Matrix<double, 1, Eigen::Dynamic>
            distances_to_first_vertex_along_graph_m_B;
        TimestampNsVector timestamp_ns_vector_B;
        extractPositionAndTimestampMatricesFromCandidateList(
            candidates_B, &G_p_G_S_matrix_B,
            &distances_to_first_vertex_along_graph_m_B, &timestamp_ns_vector_B);

        if (!searchForProximityBasedAlignmentCandidatePairsBetweenTwoMissions(
                config, mission_id_A, mission_id_B, candidates_A,
                G_p_G_S_matrix_A, distances_to_first_vertex_along_graph_m_A,
                timestamp_ns_vector_A, candidates_B, G_p_G_S_matrix_B,
                distances_to_first_vertex_along_graph_m_B,
                timestamp_ns_vector_B, candidate_pairs_ptr)) {
          return false;
        }
      }
    }
  }
  return true;
}

bool searchGloballyForAlignmentCandidatePairsBetweenTwoMissions(
    const SearchConfig& /*config*/, const vi_map::VIMap& /*map*/,
    const vi_map::MissionId& mission_A, const vi_map::MissionId& mission_B,
    const AlignmentCandidateList& /*candidates_A*/,
    const AlignmentCandidateList& /*candidates_B*/,
    AlignmentCandidatePairs* candidate_pairs_ptr) {
  CHECK_NOTNULL(candidate_pairs_ptr);

  VLOG(1) << "Searching for global candidates between missions " << mission_A
          << " and " << mission_B;

  // TODO(mfehr): implement.

  return true;
}

bool searchGloballyForAlignmentCandidatePairs(
    const SearchConfig& config, const vi_map::VIMap& map,
    const vi_map::MissionIdList& mission_ids,
    const MissionToAlignmentCandidatesMap& candidates_per_mission,
    AlignmentCandidatePairs* candidate_pairs_ptr) {
  CHECK_NOTNULL(candidate_pairs_ptr);

  vi_map::MissionIdList::const_iterator it_A = mission_ids.begin();
  for (; it_A != mission_ids.end(); ++it_A) {
    const vi_map::MissionId& mission_id_A = *it_A;
    const AlignmentCandidateList& candidates_A =
        candidates_per_mission.at(mission_id_A);

    vi_map::MissionIdList::const_iterator start_it_B = it_A;
    vi_map::MissionIdList::const_iterator end_it_B = mission_ids.end();
    // If no intra-mission candidates are desired, advance the start iterator to
    // skip the current, same mission.
    if (!config.enable_intra_mission_global_search) {
      ++start_it_B;
    }
    // If no inter-mission candidates are desired, set end iterator to the one
    // after the current, same mission.
    if (!config.enable_inter_mission_global_search) {
      end_it_B = it_A + 1;
    }

    vi_map::MissionIdList::const_iterator it_B = start_it_B;
    for (; it_B != end_it_B; ++it_B) {
      const vi_map::MissionId& mission_id_B = *it_B;
      const AlignmentCandidateList& candidates_B =
          candidates_per_mission.at(mission_id_B);

      if (!searchGloballyForAlignmentCandidatePairsBetweenTwoMissions(
              config, map, mission_id_A, mission_id_B, candidates_A,
              candidates_B, candidate_pairs_ptr)) {
        return false;
      }
    }
  }
  return true;
}

}  // namespace dense_mapping
