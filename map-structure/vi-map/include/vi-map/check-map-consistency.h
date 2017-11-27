#ifndef VI_MAP_CHECK_MAP_CONSISTENCY_H_
#define VI_MAP_CHECK_MAP_CONSISTENCY_H_

#include <posegraph/vertex.h>
#include <vi-map/mission.h>

namespace vi_map {
class VIMap;
bool isGpsReferenceVertex(
    const vi_map::VIMap& vi_map, const pose_graph::VertexId& vertex_id);
bool checkMapConsistency(const vi_map::VIMap& vi_map);
bool checkPosegraphConsistency(
    const vi_map::VIMap& vi_map, const vi_map::MissionId& mission_id);
bool checkForOrphanedPosegraphItems(const vi_map::VIMap& vi_map);
bool checkSensorConsistency(const vi_map::VIMap& vi_map);
}  // namespace vi_map
#endif  // VI_MAP_CHECK_MAP_CONSISTENCY_H_
