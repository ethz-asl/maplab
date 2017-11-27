#include "vi-map-helpers/vi-map-vertex-time-queries.h"

#include <glog/logging.h>

namespace vi_map_helpers {

VIMapVertexTimeQueries::VIMapVertexTimeQueries(const vi_map::VIMap::Ptr& map)
    : map_(map) {
  CHECK(map_);
  pose_graph::VertexIdList vertex_ids;
  map_->getAllVertexIds(&vertex_ids);
  buildVertexIdTimestampIndex(vertex_ids);
}

VIMapVertexTimeQueries::VIMapVertexTimeQueries(
    const vi_map::VIMap::Ptr& map, const vi_map::MissionId& mission_id)
    : map_(map) {
  CHECK(map_);
  CHECK(mission_id.isValid());
  CHECK(mission_id.isValid());
  pose_graph::VertexIdList vertex_ids;
  map_->getAllVertexIdsInMission(mission_id, &vertex_ids);
  buildVertexIdTimestampIndex(vertex_ids);
}

void VIMapVertexTimeQueries::buildVertexIdTimestampIndex(
    const pose_graph::VertexIdList& vertex_ids) {
  vertex_timestamp_index_.clear();
  for (const pose_graph::VertexIdList::value_type& vertex_id : vertex_ids) {
    const int64_t vertex_timestamp =
        map_->getVertex(vertex_id).getMinTimestampNanoseconds();
    vertex_timestamp_index_.addValue(vertex_timestamp, vertex_id);
  }
}

bool VIMapVertexTimeQueries::getClosestVertexInTime(
    const int64_t timestamp_nanoseconds,
    pose_graph::VertexId* vertex_id) const {
  CHECK_NOTNULL(vertex_id)->setInvalid();
  return vertex_timestamp_index_.getNearestValueToTime(
      timestamp_nanoseconds, vertex_id);
}

VIMapMissionsVertexTimeQueries::VIMapMissionsVertexTimeQueries(
    const vi_map::VIMap::Ptr& map) {
  CHECK(map);
  vi_map::MissionIdList mission_ids;
  map->getAllMissionIds(&mission_ids);
  for (const vi_map::MissionId& mission_id : mission_ids) {
    CHECK(mission_id.isValid());
    mission_to_vertex_timestamp_index_map_.emplace(
        std::piecewise_construct, std::forward_as_tuple(mission_id),
        std::forward_as_tuple(map, mission_id));
  }
}

bool VIMapMissionsVertexTimeQueries::getClosestVertexInTimeForMission(
    const int64_t timestamp_nanoseconds, const vi_map::MissionId& mission_id,
    pose_graph::VertexId* vertex_id) const {
  CHECK(mission_id.isValid());
  CHECK_NOTNULL(vertex_id)->setInvalid();
  std::unordered_map<vi_map::MissionId, VIMapVertexTimeQueries>::const_iterator
      vertex_timestamp_index_iterator =
          mission_to_vertex_timestamp_index_map_.find(mission_id);
  CHECK(vertex_timestamp_index_iterator !=
        mission_to_vertex_timestamp_index_map_.end())
      << "There is no vertex-timestamp-index available for mission "
      << mission_id.hexString();
  return vertex_timestamp_index_iterator->second.getClosestVertexInTime(
      timestamp_nanoseconds, vertex_id);
}

}  // namespace vi_map_helpers
