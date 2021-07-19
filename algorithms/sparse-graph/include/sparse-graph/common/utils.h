#ifndef SPARSE_GRAPH_DENSE_UTILS_H_
#define SPARSE_GRAPH_DENSE_UTILS_H_

#include <vi-map/vi-map.h>

#include <ros/ros.h>

namespace spg {

class Utils {
 public:
  static ros::Time CreateRosTimestamp(const int64_t ts_ns);
  static vi_map::MissionIdList GetMissionIds(
      const vi_map::VIMap* map, const pose_graph::VertexIdList& vertices);
};

}  // namespace spg

#endif  // SPARSE_GRAPH_DENSE_UTILS_H_
