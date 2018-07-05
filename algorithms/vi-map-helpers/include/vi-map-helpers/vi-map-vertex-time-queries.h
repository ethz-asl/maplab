#ifndef VI_MAP_HELPERS_VI_MAP_VERTEX_TIME_QUERIES_H_
#define VI_MAP_HELPERS_VI_MAP_VERTEX_TIME_QUERIES_H_

#include <unordered_map>

#include <maplab-common/temporal-buffer.h>
#include <posegraph/unique-id.h>
#include <vi-map/vi-map.h>

namespace vi_map_helpers {

class VIMapVertexTimeQueries {
 public:
  VIMapVertexTimeQueries() = delete;
  explicit VIMapVertexTimeQueries(const vi_map::VIMap& map);
  VIMapVertexTimeQueries(
      const vi_map::VIMap& map, const vi_map::MissionId& mission_id);
  virtual ~VIMapVertexTimeQueries() = default;

  bool getClosestVertexInTime(const int64_t timestamp_nanoseconds,
                              pose_graph::VertexId* vertex_id) const;
  bool getVertexInTimeEqualOrBefore(
      const int64_t timestamp_nanoseconds, pose_graph::VertexId* vertex_id,
      int64_t* vertex_timestamp_nanoseconds) const;
  bool getVertexInTimeEqualOrAfter(
      const int64_t timestamp_nanoseconds, pose_graph::VertexId* vertex_id,
      int64_t* vertex_timestamp_nanoseconds) const;

 private:
  void buildVertexIdTimestampIndex(
      const pose_graph::VertexIdList& vertex_ids, const vi_map::VIMap& map);

  typedef common::TemporalBuffer<pose_graph::VertexId> VertexIdTimestampIndex;
  VertexIdTimestampIndex vertex_timestamp_index_;
};

class VIMapMissionsVertexTimeQueries {
 public:
  VIMapMissionsVertexTimeQueries() = delete;
  explicit VIMapMissionsVertexTimeQueries(const vi_map::VIMap& map);
  virtual ~VIMapMissionsVertexTimeQueries() = default;

  bool getClosestVertexInTimeForMission(const int64_t timestamp_nanoseconds,
                                        const vi_map::MissionId& mission_id,
                                        pose_graph::VertexId* vertex_id) const;

  std::unordered_map<vi_map::MissionId, VIMapVertexTimeQueries>
      mission_to_vertex_timestamp_index_map_;
};

}  // namespace vi_map_helpers

#endif  // VI_MAP_HELPERS_VI_MAP_VERTEX_TIME_QUERIES_H_
