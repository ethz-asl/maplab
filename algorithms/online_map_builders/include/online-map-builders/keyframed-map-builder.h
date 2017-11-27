#ifndef ONLINE_MAP_BUILDERS_KEYFRAMED_MAP_BUILDER_H_
#define ONLINE_MAP_BUILDERS_KEYFRAMED_MAP_BUILDER_H_

#include <vector>

#include <aslam/cameras/ncamera.h>
#include <map-sparsification/keyframe-pruning.h>
#include <vi-map-helpers/vi-map-manipulation.h>
#include <vi-map/vi-map.h>
#include <vio-common/vio-update.h>

#include "online-map-builders/stream-map-builder.h"

namespace online_map_builders {

class KeyframedMapBuilder {
 public:
  KeyframedMapBuilder(
      const std::shared_ptr<aslam::NCamera>& camera_rig,
      const map_sparsification::KeyframingHeuristicsOptions& keyframing_options,
      vi_map::VIMap* map);

  // Applies the given VIO update and performs keyframing on the new map parts.
  void applyUpdateAndKeyframe(const vio::VioUpdate::ConstPtr& update);

  // Applies all the VIO updates that were held back because their keyframe
  // status couldn't be determined yet. The held back updates are marked as
  // keyframes and will stay in the map, i.e. no keyframing is done during this
  // function.
  void applyHeldBackUpdatesAndMarkAsKeyframes();

  vi_map::MissionId getMissionId() const {
    return stream_map_builder_.getMissionId();
  }

  pose_graph::VertexId getRootVertexId() const {
    return stream_map_builder_.getRootVertexId();
  }
  pose_graph::VertexId getLastVertexId() const {
    return stream_map_builder_.getLastVertexId();
  }

 private:
  void applyHeldBackUpdates();
  void keyframeAndInitLandmarks(const bool perform_keyframing);

  vi_map::VIMap* map_;
  StreamMapBuilder stream_map_builder_;

  std::vector<vio::VioUpdate::ConstPtr> new_vio_updates_;
  std::vector<vio::VioUpdate::ConstPtr> held_back_non_kf_updates_;
  pose_graph::VertexId last_keyframe_id_;
  map_sparsification::KeyframingHeuristicsOptions keyframing_options_;

  vi_map_helpers::VIMapManipulation map_manipulation_;
  vi_map_helpers::VIMapManipulation::TrackIndexToLandmarkIdMap
      track_id_to_landmark_id_map_;
};

}  // namespace online_map_builders

#endif  // ONLINE_MAP_BUILDERS_KEYFRAMED_MAP_BUILDER_H_
