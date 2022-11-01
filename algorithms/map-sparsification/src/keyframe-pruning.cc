#include "map-sparsification/keyframe-pruning.h"

#include <gflags/gflags.h>
#include <glog/logging.h>
#include <maplab-common/progress-bar.h>
#include <posegraph/unique-id.h>
#include <vi-map-helpers/vi-map-queries.h>
#include <vi-map/vi-map.h>

#include <Eigen/Core>
#include <memory>
#include <vector>

DEFINE_double(
    kf_distance_threshold_m, 0.75,
    "Distance threshold to add a new keyframe [m].");
DEFINE_double(
    kf_rotation_threshold_deg, 20,
    "Rotation threshold to add a new keyframe [deg].");
DEFINE_uint64(kf_every_nth_vertex, 10, "Force a keyframe every n-th vertex.");
DEFINE_uint64(
    kf_min_shared_landmarks_obs, 20,
    "Coobserved landmark number to add a new keyframe.");

namespace map_sparsification {
namespace {
size_t removeAllVerticesBetweenTwoVertices(
    const pose_graph::VertexId& start_kf_id,
    const pose_graph::VertexId& end_kf_id, vi_map::VIMap* map) {
  CHECK_NOTNULL(map);
  CHECK(start_kf_id.isValid());
  CHECK(end_kf_id.isValid());

  pose_graph::VertexId current_vertex_id = start_kf_id;

  // First check which edge types we can merge
  bool merge_viwls_edges = true;
  bool merge_odometry_edges = true;
  bool merge_wheel_odometry_edges = true;

  do {
    const bool edge_found =
        map->getNextVertex(current_vertex_id, &current_vertex_id);
    CHECK(edge_found)
        << "Cannot merge vertice ids " << start_kf_id << " and " << end_kf_id
        << " since no traversable path between them in the graph was found!";
    CHECK(current_vertex_id.isValid());

    // Since we iterate inclusively to the last vertex it's enough to check only
    // incoming edge types
    pose_graph::EdgeIdSet current_vertex_incoming_edge_ids;
    map->getVertex(current_vertex_id)
        .getIncomingEdges(&current_vertex_incoming_edge_ids);

    bool found_viwls_edge = false;
    bool found_odometry_edge = false;
    bool found_wheel_odometry_edge = false;
    for (const pose_graph::EdgeId& incoming_edge :
         current_vertex_incoming_edge_ids) {
      if (map->getEdgeType(incoming_edge) ==
          pose_graph::Edge::EdgeType::kViwls) {
        found_viwls_edge = true;
      }
      if (map->getEdgeType(incoming_edge) ==
          pose_graph::Edge::EdgeType::kOdometry) {
        found_odometry_edge = true;
      }
      if (map->getEdgeType(incoming_edge) ==
          pose_graph::Edge::EdgeType::kWheelOdometry) {
        found_wheel_odometry_edge = true;
      }
    }

    // If we are missing even one edge then drop the entire edge merger
    // between the two vertices. TODO(ben): improve logic to deal with
    // missing edges e.g. by interpolating using IMU measurements.
    merge_viwls_edges &= found_viwls_edge;
    merge_odometry_edges &= found_odometry_edge;
    merge_wheel_odometry_edges &= found_wheel_odometry_edge;
  } while (current_vertex_id != end_kf_id);

  // Merge consecutive vertices adding up the edges we are able to merge
  size_t num_merged_vertices = 0u;
  while (map->getNextVertex(start_kf_id, &current_vertex_id) &&
         current_vertex_id != end_kf_id) {
    map->mergeNeighboringVertices(
        start_kf_id, current_vertex_id, merge_viwls_edges, merge_odometry_edges,
        merge_wheel_odometry_edges);
    ++num_merged_vertices;
  }

  return num_merged_vertices;
}
}  // namespace

KeyframingHeuristicsOptions
KeyframingHeuristicsOptions::initializeFromGFlags() {
  KeyframingHeuristicsOptions options;
  options.kf_distance_threshold_m = FLAGS_kf_distance_threshold_m;
  options.kf_rotation_threshold_deg = FLAGS_kf_rotation_threshold_deg;
  options.kf_every_nth_vertex = FLAGS_kf_every_nth_vertex;
  options.kf_min_shared_landmarks_obs = FLAGS_kf_min_shared_landmarks_obs;
  return options;
}

void KeyframingHeuristicsOptions::serialize(
    proto::KeyframingHeuristicsOptions* proto) const {
  CHECK_NOTNULL(proto);
  proto->set_kf_distance_threshold_m(kf_distance_threshold_m);
  proto->set_kf_rotation_threshold_deg(kf_rotation_threshold_deg);
  proto->set_kf_every_nth_vertex(kf_every_nth_vertex);
  proto->set_kf_min_shared_landmarks_obs(kf_min_shared_landmarks_obs);
}

void KeyframingHeuristicsOptions::deserialize(
    const proto::KeyframingHeuristicsOptions& proto) {
  CHECK(proto.has_kf_distance_threshold_m());
  kf_distance_threshold_m = proto.kf_distance_threshold_m();
  CHECK(proto.has_kf_rotation_threshold_deg());
  kf_rotation_threshold_deg = proto.kf_rotation_threshold_deg();
  CHECK(proto.has_kf_every_nth_vertex());
  kf_every_nth_vertex = proto.kf_every_nth_vertex();
  CHECK(proto.has_kf_min_shared_landmarks_obs());
  kf_min_shared_landmarks_obs = proto.kf_min_shared_landmarks_obs();
}

size_t selectKeyframesBasedOnHeuristics(
    const vi_map::VIMap& map, const pose_graph::VertexId& start_keyframe_id,
    const pose_graph::VertexId& end_vertex_id,
    const KeyframingHeuristicsOptions& options,
    std::vector<pose_graph::VertexId>* selected_keyframes) {
  CHECK_NOTNULL(selected_keyframes)->clear();
  CHECK(start_keyframe_id.isValid());
  CHECK(end_vertex_id.isValid());

  const vi_map::MissionId& mission_id =
      map.getMissionIdForVertex(start_keyframe_id);
  CHECK_EQ(mission_id, map.getMissionIdForVertex(end_vertex_id));

  pose_graph::VertexId last_keyframe_id = start_keyframe_id;
  pose_graph::VertexId current_vertex_id = start_keyframe_id;
  size_t num_frames_since_last_keyframe = 0u;

  auto insert_keyframe = [&](const pose_graph::VertexId& keyframe_id) {
    CHECK(keyframe_id.isValid());
    num_frames_since_last_keyframe = 0u;
    last_keyframe_id = keyframe_id;
    selected_keyframes->emplace_back(keyframe_id);
  };

  // The first vertex in the range is always a keyframe.
  insert_keyframe(start_keyframe_id);

  // Traverse the posegraph and select keyframes to keep based on the
  // following conditions. The ordering of the condition evaluation is
  // important.
  //   - max. temporal spacing (force a keyframe every n-th frame)
  //   - common landmark/track count
  //   - distance and rotation threshold
  // TODO(schneith): Online-viwls should add special vio constraints (rot.
  // only, stationary) and avoid keyframes during these states.
  bool is_end_vertex_reached = false;
  while (
      map.getNextVertex(current_vertex_id, &current_vertex_id) &&
      !is_end_vertex_reached) {
    if (current_vertex_id == end_vertex_id) {
      is_end_vertex_reached = true;
    }
    // Add a keyframe every n-th frame.
    if (num_frames_since_last_keyframe >= options.kf_every_nth_vertex) {
      VLOG(3) << "Adding keyframe " << current_vertex_id
              << ". Condition: every nth vertex ("
              << num_frames_since_last_keyframe << " / "
              << options.kf_every_nth_vertex << ").";
      insert_keyframe(current_vertex_id);
      continue;
    }

    // Insert a keyframe if the common landmark observations between the last
    // keyframe and this current vertex frame drop below a certain threshold.
    vi_map_helpers::VIMapQueries queries(map);
    const size_t common_observations =
        queries.getNumberOfCommonLandmarks(current_vertex_id, last_keyframe_id);
    if (common_observations < options.kf_min_shared_landmarks_obs) {
      VLOG(3) << "Adding keyframe " << current_vertex_id
              << ". Condition: number of common landmarks ("
              << common_observations << " / "
              << options.kf_min_shared_landmarks_obs << ").";
      insert_keyframe(current_vertex_id);
      continue;
    }

    // select keyframe if there is an absolute 6DoF pose constraint
    const vi_map::Vertex& vertex = map.getVertex(current_vertex_id);
    if (vertex.getNumAbsolute6DoFMeasurements() > 0u) {
      VLOG(3) << "Adding keyframe " << current_vertex_id
              << ". Condition: has absolute 6DoF measurement.";
      insert_keyframe(current_vertex_id);
      continue;
    }

    // Select keyframe if there is an incoming or outgoing loop closure edge.
    pose_graph::EdgeIdList outgoing_edges, incoming_edges;
    map.getOutgoingOfType(
        pose_graph::Edge::EdgeType::kLoopClosure, current_vertex_id,
        &outgoing_edges);
    map.getIncomingOfType(
        pose_graph::Edge::EdgeType::kLoopClosure, current_vertex_id,
        &incoming_edges);
    if (!outgoing_edges.empty() || !incoming_edges.empty()) {
      VLOG(3) << "Adding keyframe " << current_vertex_id
              << ". Condition: has incoming or outgoing lc edges.";
      insert_keyframe(current_vertex_id);
      continue;
    }

    // Select keyframe if the distance or rotation to the last keyframe
    // exceeds a threshold.
    aslam::Transformation T_M_Bkf = map.getVertex_T_G_I(last_keyframe_id);
    aslam::Transformation T_M_Bi = map.getVertex_T_G_I(current_vertex_id);

    aslam::Transformation T_Bkf_Bi = T_M_Bkf.inverse() * T_M_Bi;
    const double distance_to_last_keyframe_m = T_Bkf_Bi.getPosition().norm();
    const double rotation_to_last_keyframe_rad =
        aslam::AngleAxis(T_Bkf_Bi.getRotation()).angle();

    if (distance_to_last_keyframe_m >= options.kf_distance_threshold_m) {
      VLOG(3) << "Adding keyframe " << current_vertex_id
              << ". Condition: distance threshold ("
              << distance_to_last_keyframe_m << " m / "
              << options.kf_distance_threshold_m << " m).";
      insert_keyframe(current_vertex_id);
      continue;
    }

    if (rotation_to_last_keyframe_rad >=
        options.kf_rotation_threshold_deg * kDegToRad) {
      VLOG(3) << "Adding keyframe " << current_vertex_id
              << ". Condition: rotation threshold ("
              << rotation_to_last_keyframe_rad * kRadToDeg << " deg / "
              << (options.kf_rotation_threshold_deg) << " deg).";
      insert_keyframe(current_vertex_id);
      continue;
    }

    ++num_frames_since_last_keyframe;
  }

  // Ensure the last vertex is added as a keyframe if it hasn't been selected
  // by any heuristics yet.
  if (last_keyframe_id != current_vertex_id) {
    insert_keyframe(current_vertex_id);
  }

  return selected_keyframes->size();
}

size_t removeVerticesBetweenKeyframes(
    const pose_graph::VertexIdList& keyframe_ids, vi_map::VIMap* map) {
  // Nothing to remove if there are less than two keyframes.
  if (keyframe_ids.size() < 2) {
    return 0u;
  }

  const vi_map::MissionId& mission_id =
      map->getMissionIdForVertex(keyframe_ids.front());

  size_t num_removed_vertices = 0u;
  const size_t num_keyframe_id_pairs = keyframe_ids.size() - 1;
  common::ProgressBar progress_bar(num_keyframe_id_pairs);
  for (size_t idx = 0u; idx < num_keyframe_id_pairs; ++idx) {
    CHECK_EQ(mission_id, map->getMissionIdForVertex(keyframe_ids[idx + 1]))
        << "All keyframes must be of the same mission.";
    num_removed_vertices += removeAllVerticesBetweenTwoVertices(
        keyframe_ids[idx], keyframe_ids[idx + 1], map);

    if (idx % 10 == 0u) {
      progress_bar.update(idx);
    }
  }
  return num_removed_vertices;
}

}  // namespace map_sparsification
