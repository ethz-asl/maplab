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
  void computeAdjacencyMatrix(const vi_map::VIMap* map);
  void publishLatestGraph(const vi_map::VIMap* map);
  std::size_t getMissionGraphSize(const std::string& map_key) const noexcept;
  std::map<uint32_t, pose_graph::VertexIdList> getAllVerticesPerSubmap() const
      noexcept;
  pose_graph::VertexIdList getVerticesForSubmap(const uint32_t submap_id) const
      noexcept;
  pose_graph::VertexIdList getAllMissionVertices() const noexcept;

  void attachResiduals(std::map<uint32_t, double>&& residuals);
  void writeResultsToFile();

 private:
  bool findMissionGraphForId(
      const uint32_t submap_id, const MissionGraph** mission_graph,
      std::string* robot_name = nullptr) const;

  std::vector<std::size_t> findVertexInGraph(
      const pose_graph::VertexId& v) const;
  pose_graph::VertexId retrieveVertex(
      const uint32_t submap_id, const uint32_t local_id) const;

  double computeDistanceBetweenNodes(
      const std::size_t i, const std::size_t j) const noexcept;
  double computeCoObservability(
      const vi_map::VIMap* map, const std::size_t i, const std::size_t j) const
      noexcept;

  double computeLoopClosureEdgeWeight(
      const std::map<pose_graph::VertexId, std::vector<pose_graph::VertexId>>&
          lc_edges,
      const std::size_t i, const std::size_t j) const noexcept;

  std::string getKeyForSubmapId(const uint32_t submap_id) const;

  std::map<pose_graph::VertexId, std::vector<pose_graph::VertexId>>
  computeLoopClosureEdgeMap(const vi_map::VIMap* map);

  bool wasSubmapPublished(const uint32_t submap_id) const;
  bool publishSubmap(
      const vi_map::VIMap* map, const RepresentativeNode& node,
      const MissionGraph& mission, const std::string& robot_name) const;
  std::vector<RepresentativeNode> getNodesForSubmap(
      const uint32_t submap_id) const;

  void publishGraphForBuilding() const;
  void publishTrajecotryForEvaluation() const;
  void publishGraphForVisualization() const;
  void publishNewSubmaps(const vi_map::VIMap* map);

  std::map<std::string, MissionGraph> mission_graphs_;
  std::atomic<uint32_t> submap_id_;
  std::vector<RepresentativeNode> sparse_graph_;
  Eigen::MatrixXd adjacency_matrix_;
  std::atomic<uint32_t> pub_seq_;
  std::vector<uint32_t> pub_submap_ids;
};

}  // namespace spg

#endif  //  SPARSE_GRAPH_SPARSE_GRAPH_H_
