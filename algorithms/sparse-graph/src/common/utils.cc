#include "sparse-graph/common/utils.h"

#include <glog/logging.h>

namespace spg {

ros::Time Utils::CreateRosTimestamp(const int64_t ts_ns) {
  CHECK_GE(ts_ns, 0);
  static constexpr uint32_t kNanosecondsPerSecond = 1e9;
  const uint64_t timestamp_u64 = static_cast<uint64_t>(ts_ns);
  const uint32_t ros_timestamp_sec = timestamp_u64 / kNanosecondsPerSecond;
  const uint32_t ros_timestamp_nsec =
      timestamp_u64 - (ros_timestamp_sec * kNanosecondsPerSecond);
  return ros::Time(ros_timestamp_sec, ros_timestamp_nsec);
}
vi_map::MissionIdList Utils::GetMissionIds(
    const vi_map::VIMap* map, const pose_graph::VertexIdList& vertices) {
  CHECK_NOTNULL(map);
  vi_map::MissionIdList mission_ids;
  for (const pose_graph::VertexId vertex_id : vertices) {
    if (!map->hasVertex(vertex_id)) {
      continue;
    }
    const vi_map::Vertex& vertex = map->getVertex(vertex_id);
    const vi_map::MissionId& mission_id = vertex.getMissionId();
    const auto it =
        std::find(mission_ids.cbegin(), mission_ids.cend(), mission_id);
    if (it == mission_ids.cend()) {
      mission_ids.emplace_back(mission_id);
    }
  }
  return mission_ids;
}

}  // namespace spg
