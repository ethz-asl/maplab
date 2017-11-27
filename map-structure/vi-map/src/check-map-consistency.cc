#include <vi-map/check-map-consistency.h>

#include <unordered_map>

#include <vi-map/vi-map.h>

namespace vi_map {

/// Checks whether a given vertex is a GPS reference vertex.
/// GPS reference vertices have no incoming edges and only GPS outgoing edges,
/// of which at least one.
bool isGpsReferenceVertex(
    const vi_map::VIMap& vi_map, const pose_graph::VertexId& vertex_id) {
  pose_graph::EdgeIdSet incoming_edges;
  pose_graph::EdgeIdSet outgoing_edges;
  vi_map.getVertex(vertex_id).getIncomingEdges(&incoming_edges);
  vi_map.getVertex(vertex_id).getOutgoingEdges(&outgoing_edges);

  if ((!incoming_edges.empty()) || (outgoing_edges.empty())) {
    return false;
  }

  for (const pose_graph::EdgeId& edge_id : outgoing_edges) {
    if (vi_map.getEdgeType(edge_id) != pose_graph::Edge::EdgeType::k6DoFGps) {
      return false;
    }
  }
  return true;
}

bool checkMapConsistency(const vi_map::VIMap& vi_map) {
  bool is_consistent = true;
  // Verify that every mission has a valid base-frame.
  LOG(INFO) << "Verifying mission base-frames...";

  const SensorManager& sensor_manager = vi_map.getSensorManager();

  vi_map::MissionIdList mission_ids;
  vi_map.getAllMissionIds(&mission_ids);

  for (const vi_map::MissionId& mission_id : mission_ids) {
    if (!vi_map.hasMission(mission_id)) {
      LOG(ERROR) << "Vi map claims to have mission " << mission_id
                 << " but then returns false when retrieving it.";
      is_consistent = false;
      continue;
    }

    const vi_map::VIMission& mission = vi_map.getMission(mission_id);
    if (mission.id() != mission_id) {
      LOG(ERROR) << "Mission provided by map for mission-id " << mission_id
                 << " has a different id internally " << mission.id();
      is_consistent = false;
      continue;
    }

    if (!mission.id().isValid()) {
      LOG(ERROR) << "Found invalid mission id ";
      is_consistent = false;
      continue;
    }
    if (!mission.getBaseFrameId().isValid()) {
      LOG(ERROR) << "Found invalid mission base frame id for mission: "
                 << mission.id().hexString();
      is_consistent = false;
      continue;
    }

    if (!vi_map.hasMissionBaseFrame(mission.getBaseFrameId())) {
      LOG(ERROR) << "Mission: " << mission.id().hexString()
                 << " claims to have the baseframe id "
                 << mission.getBaseFrameId() << " but that baseframe is not in "
                 << "the map.";
      is_consistent = false;
      continue;
    }

    if (!sensor_manager.hasNCamera(mission_id)) {
      LOG(ERROR) << "Mission " << mission_id.hexString() << " does not have "
          << "an ncamera.";
      is_consistent = false;
      continue;
    }
  }
  LOG_IF(INFO, is_consistent) << "OK.";

  LOG(INFO) << "Verifying  map vertices and edges...";
  pose_graph::VertexIdList all_vertices;
  vi_map.getAllVertexIds(&all_vertices);
  for (const pose_graph::VertexId& vertex_id : all_vertices) {
    if (!vi_map.hasVertex(vertex_id)) {
      LOG(ERROR) << "VI map claims to have vertex " << vertex_id
                 << " but then returns false when retrieving it.";
      is_consistent = false;
      continue;
    }
  }
  pose_graph::EdgeIdList all_edges;
  vi_map.getAllEdgeIds(&all_edges);
  for (const pose_graph::EdgeId& edge_id : all_edges) {
    if (!vi_map.hasEdge(edge_id)) {
      LOG(ERROR) << "VI map claims to have edge " << edge_id
                 << " but then returns false when retrieving it.";
      is_consistent = false;
      continue;
    }
  }
  LOG_IF(INFO, is_consistent) << "OK.";

  // Verify that all landmark IDs in the map are valid.
  LOG(INFO) << "Verifying landmark validity...";
  vi_map::LandmarkIdList all_landmark_ids;
  vi_map.getAllLandmarkIds(&all_landmark_ids);

  // Build a list of landmarks that we expect to find in every vertex, so
  // we can then check every vertex in batch for its landmarks which is more
  // cache efficient.
  typedef std::unordered_map<pose_graph::VertexId,
                             std::vector<vi_map::LandmarkId> >
      ExpectedVertexLandmarks;
  ExpectedVertexLandmarks vertex_expected_landmarks;

  for (const vi_map::LandmarkId& landmark_id : all_landmark_ids) {
    if (!vi_map.hasLandmark(landmark_id)) {
      LOG(ERROR) << "Vi map claims to have global landmark " << landmark_id
                 << " but then returns false when retrieving it.";
      is_consistent = false;
      continue;
    }

    const pose_graph::VertexId& storing_vertex_id =
        vi_map.getLandmarkStoreVertexId(landmark_id);

    if (!vi_map.hasVertex(storing_vertex_id)) {
      LOG(ERROR) << "Landmark: " << landmark_id
                 << " points to vertex " << storing_vertex_id
                 << " but this vertex is not in the map.";
      is_consistent = false;
      continue;
    }

    vertex_expected_landmarks[storing_vertex_id].push_back(landmark_id);
  }

  // Now check all landmarks for every vertex in batch. Iterate through vertices
  // in cache friendly order too.
  for (const pose_graph::VertexId& vertex_id : all_vertices) {
    ExpectedVertexLandmarks::const_iterator it =
        vertex_expected_landmarks.find(vertex_id);
    if (it == vertex_expected_landmarks.end()) {
      LOG(WARNING) << "Skip vertex " << vertex_id << " since no landmarks.";
      continue;
    }
    CHECK(vi_map.hasVertex(vertex_id)) << "No vertex with id "
                                       << vertex_id.hexString();
    const vi_map::Vertex& vertex = vi_map.getVertex(vertex_id);
    const vi_map::LandmarkStore& landmark_store = vertex.getLandmarks();
    for (const vi_map::LandmarkId& landmark_id : it->second) {
      if (!landmark_store.hasLandmark(landmark_id)) {
        LOG(ERROR) << "Landmark to vertex table claims that landmark: "
                   << landmark_id.hexString() << " resides in vertex "
                   << vertex_id.hexString() << " which is not the case.";
        is_consistent = false;
      }
    }

    for (const vi_map::Landmark& landmark : landmark_store) {
      if (!landmark.id().isValid()) {
        LOG(ERROR) << "Landmark stored in vertex " << vertex_id.hexString()
                   << " has an invalid ID.";
        is_consistent = false;
      } else if (!vi_map.hasLandmark(landmark.id())) {
        LOG(ERROR) << "Landmark " << landmark.id().hexString() << " stored in "
                   << "vertex " << vertex_id.hexString() << " has no entry in "
                   << "landmark index.";
        is_consistent = false;
      }
    }
  }

  LOG_IF(INFO, is_consistent) << "OK.";

  // Create a map of all vertices that have a reference to a given landmark
  // in order to check the back-reference from landmark to vertices.
  typedef std::unordered_multimap<vi_map::LandmarkId, pose_graph::VertexId>
      LandmarkToVertexMap;
  LandmarkToVertexMap landmark_observing_vertices;
  LOG(INFO) << "Building landmark observers list...";
  for (const pose_graph::VertexId& vertex_id : all_vertices) {
    if (!vi_map.hasVertex(vertex_id)) {
      LOG(ERROR) << "Vi map claims to have vertex " << vertex_id
                 << " but then returns false when retrieving it.";
      is_consistent = false;
      continue;
    }

    const vi_map::Vertex& vertex = vi_map.getVertex(vertex_id);

    for (unsigned int i = 0; i < vertex.numFrames(); ++i) {
      if (vertex.isVisualFrameSet(i)) {
        for (size_t j = 0; j < vertex.observedLandmarkIdsSize(i); ++j) {
          vi_map::LandmarkId landmark_id =
              vertex.getObservedLandmarkId(i, j);
          if (landmark_id.isValid()) {
            landmark_observing_vertices.emplace(landmark_id, vertex_id);
          }
        }
      }
    }
  }
  LOG_IF(INFO, is_consistent) << "OK.";

  LOG(INFO) << "Verifying landmark references...";
  int num_checked = 0;
  for (const pose_graph::VertexId& vertex_id : all_vertices) {
    // Existence in map verified above.
    const Vertex& vertex = vi_map.getVertex(vertex_id);

    LOG_EVERY_N(INFO, static_cast<int>(all_vertices.size()) / 10)
        << "Verifying vertices... "
        << std::round(
               static_cast<double>(num_checked + 1) / all_vertices.size() *
               100.)
        << "%";

    // Verify that the Landmark IDs in the visual frame are sane:
    // These tests look for errors that have a soft bound, the thresholds
    // here are rather arbitrary.
    // If we find two times the same landmark ID, how far are they allowed to
    // be apart in image space before we warn or fail.
    static constexpr double kImageDisparitySameLandmarkWarn = 10.;
    static constexpr double kImageDisparitySameLandmarkError = 75;
    // How often can the same landmark ID occur before we declare a map
    // inconsistent.
    static constexpr int kMaxNumSameLandmarkId = 5;
    std::unordered_map<LandmarkId, int> appearance_count;
    const int num_frames = vertex.numFrames();
    for (int i = 0; i < num_frames; ++i) {
      if (vertex.isVisualFrameSet(i)) {
        if (!vertex.getVisualFrame(i).getId().isValid()) {
          LOG(ERROR) << "Visual frame id for frame " << i << " in vertex "
                     << vertex_id << " is invalid.";
          is_consistent = false;
          continue;
        }
        const int num_observed_landmarks = vertex.observedLandmarkIdsSize(i);
        for (int j = 0; j < num_observed_landmarks; ++j) {
          LandmarkId observed_landmark_j = vertex.getObservedLandmarkId(i, j);
          if (!observed_landmark_j.isValid()) {
            continue;
          }
          ++appearance_count[observed_landmark_j];
          if (appearance_count[observed_landmark_j] > kMaxNumSameLandmarkId) {
            LOG(ERROR) << "Landmark " << observed_landmark_j << " is observed "
                       << appearance_count[observed_landmark_j]
                       << "times in the same frame (" << vertex_id
                       << ") which is considered an "
                       << "error (threshold evaluates to "
                       << kMaxNumSameLandmarkId << ").";
          }

          for (int k = j + 1; k < num_observed_landmarks; ++k) {
            LandmarkId observed_landmark_k =
                vertex.getObservedLandmarkId(i, k);
            if (!observed_landmark_k.isValid()) {
              continue;
            }
            if (observed_landmark_j != observed_landmark_k) {
              continue;
            }
            // Same landmark id, check the distance in image space.
            if (vertex.getVisualFrame(i).hasKeypointMeasurements()) {
              Eigen::Matrix<double, 2, 1> measurement_i =
                  vertex.getVisualFrame(i).getKeypointMeasurement(j);
              Eigen::Matrix<double, 2, 1> measurement_j =
                  vertex.getVisualFrame(i).getKeypointMeasurement(k);
              double distance = (measurement_i - measurement_j).norm();
              if (distance > kImageDisparitySameLandmarkError) {
                LOG(ERROR) << "Landmark " << observed_landmark_j
                           << " is observed "
                           << " twice from the same frame (" << vertex_id
                           << "), but the "
                           << "observations evaluate to ["
                           << measurement_i.transpose() << "] and ["
                           << measurement_j.transpose() << "] (distance of "
                           << distance << ") and threshold evaluates to "
                           << kImageDisparitySameLandmarkError;
                is_consistent = false;
              } else if (distance > kImageDisparitySameLandmarkWarn) {
                LOG(WARNING)
                    << "Landmark " << observed_landmark_j << " is observed "
                    << " twice from the same frame (" << vertex_id
                    << "), but the "
                    << "observations evaluate to [" << measurement_i.transpose()
                    << "] and [" << measurement_j.transpose()
                    << "] (distance of " << distance
                    << ") and threshold evaluates to "
                    << kImageDisparitySameLandmarkWarn;
              }
            }
          }
        }
      }
    }

    const vi_map::LandmarkStore& landmark_store = vertex.getLandmarks();

    for (const vi_map::Landmark& landmark : landmark_store) {
      const vi_map::LandmarkId& landmark_id = landmark.id();

      // If this landmark id is non valid, it should not be in the global map.
      if (!landmark_id.isValid()) {
        LOG(ERROR) << "Landmark " << landmark_id.hexString()
                   << " stored in vertex " << vertex_id.hexString()
                   << " is invalid. Only valid landmarks should be in the "
                      "store.";
        is_consistent = false;
      }
      // Every landmark in the store should be in the global map.
      if (!vi_map.hasLandmark(landmark_id)) {
        LOG(ERROR) << "Landmark " << landmark_id.hexString()
                   << " stored in vertex " << vertex_id.hexString()
                   << " is not found in the global map.";
        is_consistent = false;
      }

      const KeypointIdentifierList& vertex_id_frame_idx_kp_idx =
          landmark_store.getLandmark(landmark_id).getObservations();

      // Count how often we expect to find every vertex in the backwards
      // reference list of the current landmark.
      std::pair<LandmarkToVertexMap::iterator,
                LandmarkToVertexMap::iterator>
          range = landmark_observing_vertices.equal_range(landmark_id);
      std::unordered_map<pose_graph::VertexId, int> refound_map;
      for (LandmarkToVertexMap::iterator it_vertex = range.first;
           it_vertex != range.second; ++it_vertex) {
        // Increment the number of times we expect to find this to be
        // back-referenced from the landmarks.
        if (refound_map.count(it_vertex->second) == 0) {
          refound_map[it_vertex->second] = 1;
        } else {
          ++refound_map[it_vertex->second];
        }
      }

      VLOG(5) << "refound map:";
      for (const std::pair<pose_graph::VertexId, int>& observation_counts :
           refound_map) {
        VLOG(5) << "Back-references to vertex "
                << observation_counts.first.hexString() << " from landmark "
                << landmark_id.hexString() << "  " << observation_counts.second;
      }

      // Check that all other observing vertices of this landmark have
      // set the ID to the same state.
      for (unsigned int j = 0; j < vertex_id_frame_idx_kp_idx.size(); ++j) {
        const pose_graph::VertexId& observer_vertex_id =
            vertex_id_frame_idx_kp_idx[j].frame_id.vertex_id;
        if (!vi_map.hasVertex(observer_vertex_id)) {
          LOG(ERROR) << "Landmark " << landmark_id.hexString()
                     << " stored in vertex " << vertex_id
                     << " lists the vertex " << observer_vertex_id
                     << " as observer, but that vertex does not exist.";
          is_consistent = false;
        }

        const Vertex& observer_vertex = vi_map.getVertex(observer_vertex_id);

        // Check that we have seen this vertex before.
        if (refound_map.count(observer_vertex_id) == 0u) {
          LOG(ERROR) << "Landmark " << landmark_id.hexString()
                     << " has an unknown vertex listed as observer: "
                     << observer_vertex_id.hexString();
          is_consistent = false;
        }
        // Check that there is still an unmatched observation for this vertex.
        if (refound_map[observer_vertex_id] <= 0) {
          LOG(ERROR)
              << "The landmark " << landmark_id.hexString()
              << " has the vertex " << observer_vertex_id.hexString()
              << " in the list of back-references, but all back-references to"
              << " this vertex have already been matched.";
          is_consistent = false;
        }
        // Mark as found.
        --refound_map[observer_vertex_id];

        if (vertex_id_frame_idx_kp_idx[j].keypoint_index >=
            observer_vertex.observedLandmarkIdsSize(
                vertex_id_frame_idx_kp_idx[j].frame_id.frame_index)) {
          LOG(ERROR) << "Keypoint index "
                     << vertex_id_frame_idx_kp_idx[j].keypoint_index
                     << " to retrieve landmark ID "
                     << vertex_id_frame_idx_kp_idx[j].frame_id.vertex_id
                     << " is out of bounds.";
          is_consistent = false;
          continue;
        }

        const vi_map::LandmarkId& observer_landmark_id =
            observer_vertex.getObservedLandmarkId(
                vertex_id_frame_idx_kp_idx[j].frame_id.frame_index,
                vertex_id_frame_idx_kp_idx[j].keypoint_index);
        if (!observer_landmark_id.isValid()) {
          LOG(ERROR) << "The store landmark id " << landmark_id.hexString()
                     << " has a backlink to an observer that has an invalid"
                     << " landmark ID.";
          is_consistent = false;
        } else if (
            observer_landmark_id != landmark_id) {
          LOG(ERROR) << "The store vertex of landmark id "
                     << landmark_id.hexString()
                     << " and landmark id in the observer table "
                     << observer_landmark_id.hexString()
                     << " are inconsistent";
          is_consistent = false;
        }

        if (landmark_id.isValid() != observer_landmark_id.isValid()) {
          LOG(ERROR)
              << "The valid state of corresponding landmarks stored in vertex "
              << vertex_id.hexString() << " and "
              << vertex_id_frame_idx_kp_idx[j].frame_id.vertex_id.hexString()
              << " are inconsistent";
          is_consistent = false;
        }
      }

      for (const std::pair<pose_graph::VertexId, int>& observation_counts :
           refound_map) {
        if (observation_counts.second != 0) {
          LOG(ERROR) << "Back-references to vertex "
                     << observation_counts.first.hexString()
                     << " missing in the list of back-references of landmark "
                     << landmark_id.hexString();
          is_consistent = false;
        }
      }
    }
    ++num_checked;
  }
  LOG_IF(INFO, is_consistent) << "OK.";

  LOG(INFO) << "Verifying sensor consistency...";
  if (!checkSensorConsistency(vi_map)) {
    LOG(ERROR) << "Inconsistent sensors detected.";
    is_consistent = false;
  } else {
    LOG(INFO) << "OK.";
  }

  LOG(INFO) << "Verifying posegraph consistency for each mission...";
  for (const vi_map::MissionId& mission_id : mission_ids) {
    LOG(INFO) << "Verifying mission: " << mission_id << "...";
    if (!vi_map.hasMission(mission_id)) {
      LOG(ERROR) << "Vi map claims to have mission " << mission_id
                 << " but then returns false when retrieving it.";
      is_consistent = false;
      continue;
    }

    if (!checkPosegraphConsistency(vi_map, mission_id)) {
      LOG(ERROR) << "Posegraph inconsistent.";
      is_consistent = false;
    } else {
      LOG(INFO) << "OK.";
    }
  }

  LOG(INFO) << "Looking for orphaned posegraph items...";
  if (!checkForOrphanedPosegraphItems(vi_map)) {
    LOG(ERROR) << "Orphaned items detected.";
    is_consistent = false;
  } else {
    LOG(INFO) << "OK.";
  }

  LOG(INFO) << "Verifying resource consistency...";
  if (!vi_map.checkResourceConsistency()) {
    LOG(ERROR) << "Inconsistent resources or resource info entries detected.";
    is_consistent = false;
  } else {
    LOG(INFO) << "OK.";
  }

  return is_consistent;
}

bool checkPosegraphConsistency(
    const vi_map::VIMap& vi_map, const vi_map::MissionId& mission_id) {
  if (!vi_map.hasMission(mission_id)) {
    LOG(ERROR) << "Mission with ID " << mission_id << " does not exist"
               << " in the map";
    return false;
  }

  const vi_map::VIMission& mission = vi_map.getMission(mission_id);

  const pose_graph::Edge::EdgeType traversal_edge_type =
      vi_map.getGraphTraversalEdgeType(mission_id);

  const aslam::NCamera& ncamera =
      vi_map.getSensorManager().getNCameraForMission(mission_id);
  const aslam::NCameraId& mission_ncamera_id = ncamera.getId();
  CHECK(mission_ncamera_id.isValid());

  // This traverses the backbone of the posegraph.
  pose_graph::VertexId current_vertex_id = mission.getRootVertexId();
  pose_graph::VertexId previous_vertex_id;
  previous_vertex_id.setInvalid();
  do {
    CHECK(current_vertex_id.isValid());
    if (!vi_map.hasVertex(current_vertex_id)) {
      LOG(ERROR) << "Vertex with ID " << current_vertex_id << " does not exist"
                 << " in posegraph";
      return false;
    }

    // Check that the mission_id of the vertex exists and that it is equal to
    // the mission of the backbone we are traversing.
    const MissionId& vertex_mission_id =
        vi_map.getMissionIdForVertex(current_vertex_id);
    if (!vi_map.hasMission(vertex_mission_id)) {
      LOG(ERROR) << "Mission of vertex " << current_vertex_id.hexString()
                 << " not found.";
    }

    if (vertex_mission_id != mission_id) {
      LOG(ERROR) << "Vertex " << current_vertex_id.hexString()
                 << " belongs to mission " << vertex_mission_id.hexString()
                 << ", however it is connected by the backbone of mission "
                 << mission_id.hexString() << '.';
      return false;
    }

    // Check that the ncamera of this vertex is the same as the one of the
    // corresponding mission.
    aslam::NCamera::ConstPtr vertex_ncamera =
        vi_map.getVertex(current_vertex_id).getNCameras();
    CHECK(vertex_ncamera);
    if (vertex_ncamera->getId() != mission_ncamera_id) {
      LOG(ERROR) << "Vertex " << current_vertex_id.hexString()
                 << " belonging to mission " << mission_id.hexString()
                 << " does not have the correct NCamera ID ("
                 << vertex_ncamera->getId().hexString() << " vs "
                 << mission_ncamera_id.hexString() << ").";
      return false;
    }

    pose_graph::EdgeIdSet incoming_edge_ids;
    vi_map.getVertex(current_vertex_id).getIncomingEdges(&incoming_edge_ids);

    pose_graph::EdgeIdSet incoming_traversal_edge_ids;
    for (const pose_graph::EdgeId& incoming_edge_id : incoming_edge_ids) {
      // Check that the edge exists.
      if (!vi_map.hasEdge(incoming_edge_id)) {
        LOG(ERROR) << "Edge " << incoming_edge_id.hexString() << " not found.";
        return false;
      }
      const vi_map::Edge& edge =
          vi_map.getEdgeAs<vi_map::Edge>(incoming_edge_id);
      // Check that this edge points to the current vertex.
      const pose_graph::VertexId& vertex_id_to = edge.to();
      if (vertex_id_to != current_vertex_id) {
        LOG(ERROR) << "Edge " << incoming_edge_id.hexString() << " is among "
                   << "the incoming edges of vertex "
                   << current_vertex_id.hexString()
                   << ", yet it points to a different vertex: "
                   << vertex_id_to.hexString() << '.';
        return false;
      }

      // Check that the vertex this edge points from exists.
      const pose_graph::VertexId& vertex_id_from = edge.from();
      if (!vi_map.hasVertex(vertex_id_from)) {
        LOG(ERROR) << "The vertex edge " << incoming_edge_id.hexString()
                   << " points from does not exist.";
        return false;
      }

      const Edge::EdgeType edge_type = edge.getType();

      // If this is a traversal edge, check that it points from the previous
      // vertex and make sure the current vertex is not the root vertex.
      if (edge_type == traversal_edge_type) {
        if (current_vertex_id == mission.getRootVertexId()) {
          LOG(ERROR) << "Root vertex " << current_vertex_id.hexString()
                     << " has an incoming edge of traversal type.";
          return false;
        }
        CHECK(previous_vertex_id.isValid());
        if (vertex_id_from != previous_vertex_id) {
          LOG(ERROR)
              << "Incoming edge " << incoming_edge_id.hexString()
              << " is of traversal edge type "
              << static_cast<int>(traversal_edge_type)
              << ", but it does not point from the previous vertex of the "
              << "backbone pose-graph. Instead it points from vertex "
              << edge.from().hexString() << '.';
          return false;
        }
        incoming_traversal_edge_ids.emplace(incoming_edge_id);
      }
    }
    // Check that there's exactly one incoming traversal edge,
    // unless this vertex is the root vertex.
    if (current_vertex_id != mission.getRootVertexId() &&
        incoming_traversal_edge_ids.size() != 1u) {
      LOG(ERROR) << "Non-root vertex " << current_vertex_id.hexString()
                 << " has " << incoming_edge_ids.size()
                 << " incoming edges of traversal type, instead of exactly 1.";
      return false;
    }

    pose_graph::EdgeIdSet outgoing_edge_ids;
    vi_map.getVertex(current_vertex_id).getOutgoingEdges(&outgoing_edge_ids);
    pose_graph::EdgeIdSet outgoing_traversal_edge_ids;
    for (const pose_graph::EdgeId& outgoing_edge_id : outgoing_edge_ids) {
      // Check that the edge exists.
      if (!vi_map.hasEdge(outgoing_edge_id)) {
        LOG(ERROR) << "Edge " << outgoing_edge_id.hexString() << " not found.";
        return false;
      }
      const vi_map::Edge& edge =
          vi_map.getEdgeAs<vi_map::Edge>(outgoing_edge_id);
      // Check that this edge points from the current vertex.
      const pose_graph::VertexId& vertex_id_from = edge.from();
      if (vertex_id_from != current_vertex_id) {
        LOG(ERROR) << "Edge " << outgoing_edge_id.hexString() << " is among "
                   << "the outgoing edges of vertex "
                   << current_vertex_id.hexString()
                   << ", yet it points from a different vertex: "
                   << vertex_id_from.hexString() << '.';
        return false;
      }

      // Check that the vertex this edge points to exists.
      const pose_graph::VertexId& vertex_id_to = edge.to();
      if (!vi_map.hasVertex(vertex_id_to)) {
        LOG(ERROR) << "The vertex edge " << outgoing_edge_id.hexString()
                   << " points to does not exist.";
        return false;
      }

      const Edge::EdgeType edge_type = edge.getType();

      // If this is a traversal edge, check that it does not cross mission
      // boundaries.
      if (edge_type == traversal_edge_type) {
        if (vi_map.getMissionIdForVertex(vertex_id_to) != mission_id) {
          LOG(ERROR)
              << "Outgoing edge " << outgoing_edge_id.hexString()
              << " is of traversal type "
              << static_cast<int>(traversal_edge_type)
              << ", but it points to a vertex of mission "
              << vi_map.getMissionIdForVertex(vertex_id_to).hexString()
              << ", which is different than the backbone being traversed ("
              << mission_id.hexString() << ").";
          return false;
        }
        outgoing_traversal_edge_ids.emplace(outgoing_edge_id);
      }
    }
    // Check that there's at most one outgoing traversal edge.
    if (outgoing_traversal_edge_ids.size() > 1u) {
      LOG(ERROR) << "Vertex " << current_vertex_id.hexString() << " has "
                 << incoming_edge_ids.size()
                 << " outgoing edges of traversal type,  instead of at most 1.";
      return false;
    }

    previous_vertex_id = current_vertex_id;
  } while (vi_map.getNextVertex(
      current_vertex_id, traversal_edge_type, &current_vertex_id));

  return true;
}

bool checkForOrphanedPosegraphItems(const vi_map::VIMap& vi_map) {
  bool is_consistent = true;

  pose_graph::VertexIdList all_vertex_ids;
  pose_graph::EdgeIdList all_edge_ids;

  vi_map.getAllVertexIds(&all_vertex_ids);
  vi_map.getAllEdgeIds(&all_edge_ids);

  std::unordered_map<pose_graph::VertexId, bool> vertices_observed;
  std::unordered_map<pose_graph::EdgeId, bool> edges_observed;

  for (const pose_graph::VertexId& vertex_id : all_vertex_ids) {
    vertices_observed.emplace(vertex_id, false);
  }

  for (const pose_graph::EdgeId& edge_id : all_edge_ids) {
    edges_observed.emplace(edge_id, false);
  }

  // Traverse posegraph for each mission.
  vi_map::MissionIdList mission_ids;
  vi_map.getAllMissionIds(&mission_ids);
  for (const vi_map::MissionId& mission_id : mission_ids) {
    CHECK(mission_id.isValid());

    pose_graph::VertexId current_vertex_id =
        vi_map.getMission(mission_id).getRootVertexId();
    CHECK(vi_map.hasVertex(current_vertex_id));

    do {
      if (vertices_observed[current_vertex_id] == true) {
        // This vertex ID was observed before, so most probably we are
        // (infinitely) looping through the graph. Let's bail out.
        LOG(ERROR) << "Vertex " << current_vertex_id << " is observed "
                   << "the second time, most probably there is a loop in the "
                   << "posegraph.";
        return false;
      }

      vertices_observed[current_vertex_id] = true;

      // Mark incoming edge as present.
      std::unordered_set<pose_graph::EdgeId> incoming_edges;
      vi_map.getVertex(current_vertex_id).getIncomingEdges(&incoming_edges);
      for (const pose_graph::EdgeId& incoming_edge_id : incoming_edges) {
        edges_observed[incoming_edge_id] = true;
      }
    } while (vi_map.getNextVertex(
        current_vertex_id, vi_map.getGraphTraversalEdgeType(mission_id),
        &current_vertex_id));
  }

  // Look for orphaned elements.
  for (const std::pair<pose_graph::VertexId, bool>& vertex :
       vertices_observed) {
    // If this vertex was not traversed and it's not a GPS reference vertex,
    // it is orphaned.
    if ((!vertex.second) && (!isGpsReferenceVertex(vi_map, vertex.first))) {
      LOG(ERROR)
          << "Vertex with ID " << vertex.first << " does not belong "
          << "to any trajectory, but is present on the posegraph. It is most "
          << "likely orphaned.";
      is_consistent = false;
    }
  }

  for (const std::pair<pose_graph::EdgeId, bool>& edge : edges_observed) {
    if (!edge.second) {
      LOG(ERROR)
          << "Edge with ID " << edge.first << " does not "
          << "belong to any trajectory, but it is present on the posegraph. "
          << "It is most probably orphaned.";
      is_consistent = false;
    }
  }

  return is_consistent;
}

bool checkSensorConsistency(const vi_map::VIMap& vi_map) {
  const SensorManager& sensor_manager = vi_map.getSensorManager();

  MissionIdList mission_ids;
  vi_map.getAllMissionIds(&mission_ids);

  for (const MissionId& mission_id : mission_ids) {
    CHECK(mission_id.isValid());
    const aslam::NCamera& mission_ncamera =
        sensor_manager.getNCameraForMission(mission_id);
    const aslam::NCameraId& mission_ncamera_id = mission_ncamera.getId();
    if (!mission_ncamera_id.isValid()) {
      LOG(ERROR) << "Mission " << mission_id.hexString()
          << " has an invalid ncamera id.";
      return false;
    }

    if (!sensor_manager.hasNCamera(mission_ncamera_id)) {
      LOG(ERROR) << "The ncamera with id " << mission_ncamera_id.hexString()
          << " referenced by mission " << mission_id.hexString()
          << " is not present in the sensor manager of the corresponding map.";
      return false;
    }

    if (vi_map.hasOptionalSensorData(mission_id)) {
      const OptionalSensorData& optional_sensor_data =
          vi_map.getOptionalSensorData(mission_id);
      SensorIdSet sensor_ids_with_optional_sensor_data;
      optional_sensor_data.getAllSensorIds(
          &sensor_ids_with_optional_sensor_data);
      for (const SensorId& sensor_id_with_optional_sensor_data :
          sensor_ids_with_optional_sensor_data) {
        CHECK(sensor_id_with_optional_sensor_data.isValid());

        if (!sensor_manager.hasSensor(sensor_id_with_optional_sensor_data)) {
          LOG(ERROR)
              << "The sensor with id "
              << sensor_id_with_optional_sensor_data.hexString()
              << " has optional data stored in mission "
              << mission_id.hexString()
              << ", yet, the sensor id is not present in the sensor manager of "
              << "the corresponding map.";
          return false;
        }
      }
    }
  }
  return true;
}

}  // namespace vi_map
