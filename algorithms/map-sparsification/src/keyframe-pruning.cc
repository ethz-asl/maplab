#include "map-sparsification/keyframe-pruning.h"

#include <memory>
#include <vector>

#include <Eigen/Core>
#include <gflags/gflags.h>
#include <glog/logging.h>
#include <posegraph/unique-id.h>
#include <vi-map-helpers/vi-map-queries.h>
#include <vi-map/vi-map.h>

DEFINE_double(kf_distance_threshold_m, 0.75,
              "Distance threshold to add a new keyframe [m].");
DEFINE_double(kf_rotation_threshold_deg, 20,
              "Rotation threshold to add a new keyframe [deg].");
DEFINE_uint64(kf_every_nth_vertex, 10, "Force a keyframe every n-th vertex.");
DEFINE_uint64(kf_min_shared_landmarks_obs, 20,
              "Coobserved landmark number to add a new keyframe.");

namespace map_sparsification {
namespace {
size_t removeAllVerticesBetweenTwoVertices(
    pose_graph::Edge::EdgeType traversal_edge,
    const pose_graph::VertexId& start_kf_id,
    const pose_graph::VertexId& end_kf_id,
    vi_map::VIMap* map) {
  CHECK_NOTNULL(map);
  CHECK(traversal_edge != pose_graph::Edge::EdgeType::kUndefined);
  CHECK(start_kf_id.isValid());
  CHECK(end_kf_id.isValid());

  size_t num_merged_vertices = 0u;
  pose_graph::VertexId current_vertex_id;
  while (map->getNextVertex(start_kf_id, traversal_edge, &current_vertex_id) &&
         current_vertex_id != end_kf_id) {
    CHECK(current_vertex_id.isValid());
    CHECK(start_kf_id != current_vertex_id);
    map->mergeNeighboringVertices(start_kf_id, current_vertex_id);
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
  pose_graph::Edge::EdgeType backbone_type =
      map.getGraphTraversalEdgeType(mission_id);

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

  // Traverse the posegraph and select keyframes to keep based on the following
  // conditions.
  // The ordering of the condition evaluation is important.
  //   - max. temporal spacing (force a keyframe every n-th frame)
  //   - common landmark/track count
  //   - distance and rotation threshold
  // TODO(schneith): Online-viwls should add special vio constraints (rot. only,
  // stationary) and avoid keyframes during these states.
  while (map.getNextVertex(current_vertex_id, backbone_type,
                           &current_vertex_id)) {
    // Add a keyframe every n-th frame.
    if (num_frames_since_last_keyframe >= options.kf_every_nth_vertex) {
      insert_keyframe(current_vertex_id);
      continue;
    }

    // Insert a keyframe if the common landmark observations between the last
    // keyframe and this current vertex frame drop below a certain threshold.
    vi_map_helpers::VIMapQueries queries(map);
    const size_t common_observations =
        queries.getNumberOfCommonLandmarks(current_vertex_id, last_keyframe_id);
    if (common_observations < options.kf_min_shared_landmarks_obs) {
      insert_keyframe(current_vertex_id);
      continue;
    }

    // Select keyframe if the distance or rotation to the last keyframe exceeds
    // a threshold.
    aslam::Transformation T_M_Bkf = map.getVertex_T_G_I(last_keyframe_id);
    aslam::Transformation T_M_Bi = map.getVertex_T_G_I(current_vertex_id);

    aslam::Transformation T_Bkf_Bi = T_M_Bkf.inverse() * T_M_Bi;
    const double distance_to_last_keyframe_m = T_Bkf_Bi.getPosition().norm();
    const double rotation_to_last_keyframe_rad =
        aslam::AngleAxis(T_Bkf_Bi.getRotation()).angle();

    if (distance_to_last_keyframe_m >= options.kf_distance_threshold_m ||
        rotation_to_last_keyframe_rad >=
        options.kf_rotation_threshold_deg * kDegToRad) {
      insert_keyframe(current_vertex_id);
      continue;
    }

    ++num_frames_since_last_keyframe;
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
  pose_graph::Edge::EdgeType traversal_edge =
      map->getGraphTraversalEdgeType(mission_id);

  size_t num_removed_vertices = 0u;
  for (size_t idx = 0u; idx < keyframe_ids.size() - 1; ++idx) {
    CHECK_EQ(mission_id, map->getMissionIdForVertex(keyframe_ids[idx + 1]))
        << "All keyframes must be of the same mission.";
    num_removed_vertices += removeAllVerticesBetweenTwoVertices(
        traversal_edge, keyframe_ids[idx], keyframe_ids[idx + 1], map);
  }
  return num_removed_vertices;
}

}  // namespace map_sparsification
