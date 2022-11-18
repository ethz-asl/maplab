#ifndef MAP_ANCHORING_MAP_ANCHORING_H_
#define MAP_ANCHORING_MAP_ANCHORING_H_

#include <loop-closure-handler/loop-detector-node.h>
#include <vi-map/vi-map.h>
#include <visualization/viwls-graph-plotter.h>

namespace map_anchoring {

void setMissionBaseframeToKnownIfHasAbs6DoFConstraints(vi_map::VIMap* map);

void removeOutliersInAbsolute6DoFConstraints(vi_map::VIMap* map);

void setMissionBaseframeKnownState(
    const vi_map::MissionId& mission_id, const bool baseframe_known_state,
    vi_map::VIMap* map);

bool anchorMission(const vi_map::MissionId& mission_id, vi_map::VIMap* map);
bool anchorAllMissions(vi_map::VIMap* map);
bool anchorAllMissions(
    vi_map::VIMap* map, const visualization::ViwlsGraphRvizPlotter* plotter);

bool addAllMissionsWithKnownBaseFrameToProvidedLoopDetector(
    const vi_map::VIMap& map,
    loop_detector_node::LoopDetectorNode* loop_detector);
bool anchorMissionUsingProvidedLoopDetector(
    const vi_map::MissionId& mission_id,
    const loop_detector_node::LoopDetectorNode& loop_detector,
    vi_map::VIMap* map);

bool gravityAlignMission(
    const vi_map::MissionId& mission_id, vi_map::VIMap* map);
bool gravityAlignAllMissions(vi_map::VIMap* map);
}  // namespace map_anchoring

#endif  // MAP_ANCHORING_MAP_ANCHORING_H_
