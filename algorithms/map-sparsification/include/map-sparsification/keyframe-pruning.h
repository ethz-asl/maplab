#ifndef MAP_SPARSIFICATION_KEYFRAME_PRUNING_H_
#define MAP_SPARSIFICATION_KEYFRAME_PRUNING_H_
#include <vector>

#include <posegraph/unique-id.h>
#include <vi-map/vi-map.h>

#include "map-sparsification/keyframing-heuristics-options.pb.h"

namespace map_sparsification {

struct KeyframingHeuristicsOptions {
  double kf_distance_threshold_m;
  double kf_rotation_threshold_deg;
  size_t kf_every_nth_vertex;
  size_t kf_min_shared_landmarks_obs;

  static KeyframingHeuristicsOptions initializeFromGFlags();

  void serialize(proto::KeyframingHeuristicsOptions* proto) const;
  void deserialize(const proto::KeyframingHeuristicsOptions& proto);
};

// Select and return keyframes along the viwls-backbone on the pose graph
// between the begin and end vertex. It is assumed that the begin vertex
// is a keyframe but the last is not necessarily a keyframe.
size_t selectKeyframesBasedOnHeuristics(
    const vi_map::VIMap& map,
    const pose_graph::VertexId& last_keyframe_id,
    const pose_graph::VertexId& end_vertex_id,
    const KeyframingHeuristicsOptions& options,
    std::vector<pose_graph::VertexId>* selected_keyframes);

// Discard all vertices between keyframes and just discards all visual
// information contained in these frames. The IMU measurements of the edges
// will be concatenated into a new edge.
size_t removeVerticesBetweenKeyframes(
    const pose_graph::VertexIdList& keyframe_ids, vi_map::VIMap* map);

}  // namespace map_sparsification
#endif  // MAP_SPARSIFICATION_KEYFRAME_PRUNING_H_
