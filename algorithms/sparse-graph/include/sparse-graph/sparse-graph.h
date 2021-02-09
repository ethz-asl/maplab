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
  void publishLatestGraphWithCovs(const std::vector<Eigen::MatrixXd>& covs);
  std::size_t getMissionGraphSize(const std::string& map_key) const noexcept;
  std::map<uint32_t, pose_graph::VertexIdList> getAllVerticesPerSubmap() const
      noexcept;
  pose_graph::VertexIdList getAllMissionVertices() const noexcept;

  void attachResiduals(std::map<uint32_t, double>&& residuals);
  void writeResultsToFile();

 private:
  ros::Time createRosTimestamp(const int64_t ts_ns) const;
  bool findMissionGraphForId(
      const uint32_t submap_id, const MissionGraph** mission_graph) const;

  std::map<std::string, MissionGraph> mission_graphs_;
  std::atomic<uint32_t> submap_id_;
  std::vector<RepresentativeNode> sparse_graph_;
};

}  // namespace spg

#endif  //  SPARSE_GRAPH_SPARSE_GRAPH_H_
