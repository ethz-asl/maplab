#include "map-sparsification-plugin/keyframe-pruning.h"

#include <string>
#include <vector>

#include <console-common/console.h>
#include <map-manager/map-manager.h>
#include <map-sparsification/keyframe-pruning.h>
#include <vi-map/vi-map.h>
#include <visualization/viwls-graph-plotter.h>

namespace map_sparsification_plugin {

int keyframeMapBasedOnHeuristics(
    const map_sparsification::KeyframingHeuristicsOptions& options,
    const vi_map::MissionId& mission_id,
    visualization::ViwlsGraphRvizPlotter* plotter, vi_map::VIMap* map) {
  // plotter is optional.
  CHECK_NOTNULL(map);
  CHECK(mission_id.isValid());

  const size_t num_initial_vertices = map->numVerticesInMission(mission_id);

  pose_graph::VertexId root_vertex_id =
      map->getMission(mission_id).getRootVertexId();
  CHECK(root_vertex_id.isValid());
  pose_graph::VertexId last_vertex_id =
      map->getLastVertexIdOfMission(mission_id);

  // Select keyframes along the mission. Unconditionally add the last vertex as
  // a keyframe if it isn't a keyframe already.
  pose_graph::VertexIdList keyframe_ids;
  map_sparsification::selectKeyframesBasedOnHeuristics(
      *map, root_vertex_id, last_vertex_id, options, &keyframe_ids);
  if (keyframe_ids.empty()) {
    LOG(ERROR) << "No keyframes found.";
    return common::CommandStatus::kUnknownError;
  }

  if (keyframe_ids.back() != last_vertex_id) {
    keyframe_ids.emplace_back(last_vertex_id);
  }

  // Optionally, visualize the selected keyframes.
  if (plotter != nullptr) {
    std::vector<pose_graph::VertexIdList> partitions;
    partitions.emplace_back(keyframe_ids);
    plotter->plotPartitioning(*map, partitions);
    LOG(INFO) << "Selected " << keyframe_ids.size() << " keyframes of "
              << num_initial_vertices << " vertices.";
  }

  // Remove non-keyframe vertices.
  const size_t num_removed_keyframes =
      map_sparsification::removeVerticesBetweenKeyframes(keyframe_ids, map);
  LOG(INFO) << "Removed " << num_removed_keyframes << " vertices of "
            << num_initial_vertices << " vertices.";
  return common::CommandStatus::kSuccess;
}

}  // namespace map_sparsification_plugin
