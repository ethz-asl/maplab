#include "online-map-builders/keyframed-map-builder.h"

namespace online_map_builders {

KeyframedMapBuilder::KeyframedMapBuilder(
    const std::shared_ptr<aslam::NCamera>& camera_rig,
    const map_sparsification::KeyframingHeuristicsOptions& keyframing_options,
    vi_map::VIMap* map)
    : map_(CHECK_NOTNULL(map)),
      stream_map_builder_(camera_rig, map),
      keyframing_options_(keyframing_options),
      map_manipulation_(map) {}

void KeyframedMapBuilder::applyUpdateAndKeyframe(
    const vio::VioUpdate::ConstPtr& update) {
  CHECK(update != nullptr);
  // Apply held back keyframes.
  applyHeldBackUpdates();
  stream_map_builder_.apply(*update);
  new_vio_updates_.emplace_back(update);
  static constexpr bool kPerformKeyframing = true;
  keyframeAndInitLandmarks(kPerformKeyframing);
}

void KeyframedMapBuilder::applyHeldBackUpdatesAndMarkAsKeyframes() {
  applyHeldBackUpdates();
  static constexpr bool kPerformKeyframing = false;
  keyframeAndInitLandmarks(kPerformKeyframing);
}

void KeyframedMapBuilder::applyHeldBackUpdates() {
  for (const vio::VioUpdate::ConstPtr& held_back_update :
       held_back_non_kf_updates_) {
    stream_map_builder_.apply(*held_back_update);
    new_vio_updates_.emplace_back(held_back_update);
  }
  held_back_non_kf_updates_.clear();
}

void KeyframedMapBuilder::keyframeAndInitLandmarks(
    const bool perform_keyframing) {
  pose_graph::VertexId start_vertex_id = last_keyframe_id_;
  bool first_keyframing_and_initialization = false;
  if (!start_vertex_id.isValid()) {
    start_vertex_id = stream_map_builder_.getRootVertexId();
    first_keyframing_and_initialization = true;
  }

  pose_graph::VertexIdList selected_keyframes;
  if (perform_keyframing) {
    map_sparsification::selectKeyframesBasedOnHeuristics(
        *map_, start_vertex_id, stream_map_builder_.getLastVertexId(),
        keyframing_options_, &selected_keyframes);
    last_keyframe_id_ = selected_keyframes.back();

    const size_t num_deleted_vertices_between_keyframes =
        map_sparsification::removeVerticesBetweenKeyframes(
            selected_keyframes, map_);

    pose_graph::VertexIdList removed_vertices_at_end;
    stream_map_builder_.removeAllVerticesAfterVertexId(
        last_keyframe_id_, &removed_vertices_at_end);
    VLOG(10) << "Removed " << num_deleted_vertices_between_keyframes
             << " vertices between keyframes and "
             << removed_vertices_at_end.size()
             << " vertices at the end of the map.";

    // Push vertices back to list with vertices that haven't been keyframed yet.
    held_back_non_kf_updates_.insert(
        held_back_non_kf_updates_.end(),
        new_vio_updates_.end() - removed_vertices_at_end.size(),
        new_vio_updates_.end());

    if (!first_keyframing_and_initialization) {
      // Landmarks were already initialized for the first keyframe in the list
      // the last time.
      selected_keyframes.erase(selected_keyframes.begin());
    }
  } else {
    // Go through the map to get a list of all new keyframes.
    if (start_vertex_id.isValid()) {
      vi_map_helpers::VIMapQueries queries(*map_);
      constexpr bool kIncludeStartingVertex = false;
      queries.getFollowingVertexIdsAlongGraph(
          start_vertex_id, kIncludeStartingVertex, &selected_keyframes);
    } else {
      map_->getAllVertexIdsInMissionAlongGraph(
          stream_map_builder_.getMissionId(), &selected_keyframes);
    }
  }

  new_vio_updates_.clear();
  if (selected_keyframes.empty()) {
    return;
  }

  // Initialize landmarks.
  map_manipulation_.initializeLandmarksFromUnusedFeatureTracksOfOrderedVertices(
      selected_keyframes, &track_id_to_landmark_id_map_);
}

}  // namespace online_map_builders
