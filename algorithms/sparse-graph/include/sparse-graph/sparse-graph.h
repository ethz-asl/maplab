#ifndef SPARSE_GRAPH_SPARSE_GRAPH_H_
#define SPARSE_GRAPH_SPARSE_GRAPH_H_

#include <atomic>
#include <map>
#include <set>
#include <string>
#include <vector>

#include <ros/ros.h>
#include <vi-map/vi-map.h>

#include "sparse-graph/common/representative-node.h"
#include "sparse-graph/mission-graph.h"
#include "sparse-graph/partitioners/base-partitioner.h"

namespace spg {

class SparseGraph {
 public:
  SparseGraph();
  void addVerticesToMissionGraph(
      const std::string& map_key, const pose_graph::VertexIdList& vertices);
  void compute(const vi_map::VIMap* map, BasePartitioner* partitioner);
  void publishLatestGraph();
  std::size_t getMissionGraphSize(const std::string& map_key) const noexcept;

 private:
  ros::Time createRosTimestamp(const int64_t ts_ns) const;

  std::map<std::string, MissionGraph> mission_graphs_;
  std::atomic<uint32_t> submap_id_;
  std::set<RepresentativeNode> sparse_graph_;
};

}  // namespace spg

#endif  //  SPARSE_GRAPH_SPARSE_GRAPH_H_
