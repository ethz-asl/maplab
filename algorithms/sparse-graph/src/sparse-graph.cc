#include "sparse-graph/sparse-graph.h"

#include <fstream>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <glog/logging.h>
#include <nav_msgs/Path.h>

#include <maplab_msgs/DenseNode.h>
#include <maplab_msgs/Graph.h>
#include <maplab_msgs/Submap.h>
#include <maplab_msgs/Trajectory.h>
#include <maplab_msgs/TrajectoryNode.h>
#include <minkindr_conversions/kindr_msg.h>
#include <vi-map-helpers/vi-map-nearest-neighbor-lookup.h>
#include <vi-map-helpers/vi-map-queries.h>
#include <visualization/common-rviz-visualization.h>
#include <visualization/rviz-visualization-sink.h>

#include "sparse-graph/common/utils.h"
#include "sparse-graph/dense-map-builder.h"

namespace spg {

SparseGraph::SparseGraph() : submap_id_(0u), pub_seq_(0u) {}

void SparseGraph::addVerticesToMissionGraph(
    const std::string& map_key, const pose_graph::VertexIdList& vertices) {
  mission_graphs_[map_key].addNewVertices(submap_id_++, vertices);
}

void SparseGraph::compute(
    const vi_map::VIMap* map, BasePartitioner* partitioner) {
  CHECK_NOTNULL(partitioner);
  CHECK_NOTNULL(map);
  sparse_graph_.clear();
  for (const auto& mission_graph : mission_graphs_) {
    RepresentativeNodeVector mission =
        mission_graph.second.computeSparseGraph(partitioner);
    sparse_graph_.insert(sparse_graph_.end(), mission.begin(), mission.end());
  }
  std::sort(sparse_graph_.begin(), sparse_graph_.end());
}

void SparseGraph::publishLatestGraph(const vi_map::VIMap* map) {
  CHECK_NOTNULL(map);

  publishGraphForVisualization();
  publishGraphForBuilding();
  publishTrajecotryForEvaluation();
  publishNewSubmaps(map);

  ++pub_seq_;
}

std::size_t SparseGraph::getMissionGraphSize(const std::string& map_key) const
    noexcept {
  auto mission_it = mission_graphs_.find(map_key);
  if (mission_it == mission_graphs_.end()) {
    LOG(WARNING) << "Unknown map key " << map_key << " provided.";
    return 0;
  }
  return mission_it->second.size();
}

void SparseGraph::computeAdjacencyMatrix(const vi_map::VIMap* map) {
  CHECK_NOTNULL(map);
  const std::size_t n_nodes = sparse_graph_.size();
  if (n_nodes == 0) {
    return;
  }

  // Create nearest neighbor database for retrieval.
  adjacency_matrix_ = Eigen::MatrixXd::Zero(n_nodes, n_nodes);
  vi_map_helpers::VIMapNearestNeighborLookupVertexId nn_query_database(*map);
  const double search_radius = 6.0;

  // Create LC edge map.
  std::map<pose_graph::VertexId, std::vector<pose_graph::VertexId>> lc_edges =
      computeLoopClosureEdgeMap(map);

  // Iterate over the sparse graph.
  for (std::size_t i = 0u; i < n_nodes; ++i) {
    const RepresentativeNode& node = sparse_graph_.at(i);
    const aslam::Transformation& T_G_I = node.getPose();

    pose_graph::VertexIdSet vertex_ids_within_search_radius;
    nn_query_database.getAllDataItemsWithinRadius(
        T_G_I.getPosition(), search_radius, &vertex_ids_within_search_radius);

    // Iterate over the closest neighbors.
    for (const pose_graph::VertexId& v : vertex_ids_within_search_radius) {
      const std::vector<std::size_t> ids = findVertexInGraph(v);

      // Compute the weighted adjacency for each vertex.
      for (const std::size_t j : ids) {
        if (i == j) {
          continue;
        }
        const double w_d = computeDistanceBetweenNodes(i, j);
        const double w_c = computeCoObservability(map, i, j);
        const double w_l = computeLoopClosureEdgeWeight(lc_edges, i, j);

        // Ensure that the weights are normalized.
        CHECK(w_d >= 0.0 && w_d <= 1.0);
        CHECK(w_c >= 0.0 && w_c <= 1.0);
        CHECK(w_l >= 0.0 && w_l <= 1.0);

        // Set the weights for the adjacency
        // which is a symmetric and undirected adjacency matrix.
        adjacency_matrix_(i, j) = w_d + w_c + w_l;
        adjacency_matrix_(j, i) = adjacency_matrix_(i, j);
      }
    }
  }
}

std::map<pose_graph::VertexId, std::vector<pose_graph::VertexId>>
SparseGraph::computeLoopClosureEdgeMap(const vi_map::VIMap* map) {
  CHECK_NOTNULL(map);
  vi_map::MissionIdList mission_ids;
  map->getAllMissionIds(&mission_ids);
  std::map<pose_graph::VertexId, std::vector<pose_graph::VertexId>> lc_edges;
  for (const vi_map::MissionId& mission_id : mission_ids) {
    pose_graph::EdgeIdList edges;
    map->getAllEdgeIdsInMissionAlongGraph(
        mission_id, pose_graph::Edge::EdgeType::kLoopClosure, &edges);
    for (const pose_graph::EdgeId& edge_id : edges) {
      CHECK(edge_id.isValid());

      CHECK(map->hasEdge(edge_id));
      const vi_map::LoopClosureEdge& edge =
          map->getEdgeAs<vi_map::LoopClosureEdge>(edge_id);

      lc_edges[edge.from()].emplace_back(edge.to());
      lc_edges[edge.to()].emplace_back(edge.from());
    }
  }
  return lc_edges;
}

std::vector<std::size_t> SparseGraph::findVertexInGraph(
    const pose_graph::VertexId& v) const {
  std::vector<std::size_t> sparse_graph_ids;
  const std::size_t n_nodes = sparse_graph_.size();
  for (std::size_t i = 0u; i < n_nodes; ++i) {
    const RepresentativeNode& node = sparse_graph_.at(i);
    const uint32_t id = node.getAssociatedSubmapId();

    // Find submap for the id.
    const MissionGraph* mission_graph;
    if (!findMissionGraphForId(id, &mission_graph)) {
      LOG(WARNING) << "Found no mission graph for id = " << id;
      continue;
    }
    CHECK_NOTNULL(mission_graph);

    if (!mission_graph->containsVertex(id, v)) {
      continue;
    }

    const uint32_t local_id = mission_graph->getLocalVertexId(id, v);
    if (!node.containsLocalIndex(local_id)) {
      continue;
    }
    sparse_graph_ids.emplace_back(i);
  }
  return sparse_graph_ids;
}

pose_graph::VertexId SparseGraph::retrieveVertex(
    const uint32_t submap_id, const uint32_t local_id) const {
  const MissionGraph* mission_graph;
  findMissionGraphForId(submap_id, &mission_graph);
  CHECK_NOTNULL(mission_graph);
  return mission_graph->getVertex(submap_id, local_id);
}

double SparseGraph::computeDistanceBetweenNodes(
    const std::size_t i, const std::size_t j) const noexcept {
  const std::size_t n_nodes = sparse_graph_.size();
  if (i > n_nodes || j > n_nodes) {
    return 0.0;
  }
  const double sigma = 1.0;
  const RepresentativeNode& node_i = sparse_graph_.at(i);
  const RepresentativeNode& node_j = sparse_graph_.at(j);
  const double distance =
      (node_i.getPose().getPosition() - node_j.getPose().getPosition())
          .lpNorm<2>();

  return std::exp(-distance / (2.0 * sigma * sigma));
}

double SparseGraph::computeCoObservability(
    const vi_map ::VIMap* map, const std::size_t i, const std::size_t j) const
    noexcept {
  CHECK_NOTNULL(map);
  const std::size_t n_nodes = sparse_graph_.size();
  if (i > n_nodes || j > n_nodes) {
    return 0.0;
  }
  const RepresentativeNode& node_i = sparse_graph_.at(i);
  const RepresentativeNode& node_j = sparse_graph_.at(j);
  const vi_map_helpers::VIMapQueries queries(*map);

  // Iterate over all possible local indices.
  const uint32_t submap_id_i = node_i.getAssociatedSubmapId();
  const uint32_t submap_id_j = node_j.getAssociatedSubmapId();
  double accumulated_coobs = 0.0;
  for (const uint32_t i_local_idx : node_i.getLocalIndex()) {
    pose_graph::VertexId v_i = retrieveVertex(submap_id_i, i_local_idx);
    for (const uint32_t j_local_idx : node_j.getLocalIndex()) {
      pose_graph::VertexId v_j = retrieveVertex(submap_id_j, j_local_idx);
      const int n_obs = queries.getNumberOfCommonLandmarks(v_i, v_j);
      accumulated_coobs += static_cast<double>(n_obs);
    }
  }
  const double lambda = 0.03;

  return 1 - std::pow(std::exp(-lambda), accumulated_coobs);
}

double SparseGraph::computeLoopClosureEdgeWeight(
    const std::map<pose_graph::VertexId, std::vector<pose_graph::VertexId>>&
        lc_edges,
    const std::size_t i, const std::size_t j) const noexcept {
  if (lc_edges.empty()) {
    return 0.0;
  }
  const std::size_t n_nodes = sparse_graph_.size();
  if (i > n_nodes || j > n_nodes) {
    return 0.0;
  }
  const RepresentativeNode& node_i = sparse_graph_.at(i);
  const RepresentativeNode& node_j = sparse_graph_.at(j);
  bool found_lc = false;
  const uint32_t submap_id_i = node_i.getAssociatedSubmapId();
  const uint32_t submap_id_j = node_j.getAssociatedSubmapId();
  for (const uint32_t i_local_idx : node_i.getLocalIndex()) {
    pose_graph::VertexId v_i = retrieveVertex(submap_id_i, i_local_idx);
    if (lc_edges.find(v_i) == lc_edges.end()) {
      continue;
    }

    // Vertex v_i definitely has LC edges.
    // Check where the point to.
    const std::vector<pose_graph::VertexId>& lc_edges_to = lc_edges.at(v_i);
    for (const uint32_t j_local_idx : node_j.getLocalIndex()) {
      pose_graph::VertexId v_j = retrieveVertex(submap_id_j, j_local_idx);
      if (std::find(lc_edges_to.begin(), lc_edges_to.end(), v_j) ==
          lc_edges_to.end()) {
        continue;
      }
      found_lc = true;
    }
  }
  if (!found_lc) {
    return 0.0;
  }

  // Compute the distance between the nodes.
  const double max_dist = 10.0;
  const double distance = std::min(
      (node_i.getPose().getPosition() - node_j.getPose().getPosition())
          .lpNorm<2>(),
      max_dist);
  const double eta = 0.01;
  return 1 - std::pow(distance, 2.0) * eta;
}

bool SparseGraph::findMissionGraphForId(
    const uint32_t submap_id, const MissionGraph** mission_graph) const {
  CHECK_NOTNULL(mission_graph);
  *mission_graph = nullptr;
  for (const auto& name_and_mission_graph : mission_graphs_) {
    if (name_and_mission_graph.second.containsSubmap(submap_id)) {
      *mission_graph = &name_and_mission_graph.second;
      return true;
    }
  }
  return false;
}

void SparseGraph::writeResultsToFile() {
  ros::Time ts_ros;
  const std::string graph_out_path = "/tmp/cdpgo_graph.csv";
  const std::string signal_out_path = "/tmp/cdpgo_signal.csv";
  const std::string adj_out_path = "/tmp/cdpgo_adj.csv";
  std::ofstream fs_graph(graph_out_path, std::ofstream::trunc);
  std::ofstream fs_signal(signal_out_path, std::ofstream::trunc);
  std::ofstream fs_adj(adj_out_path, std::ofstream::trunc);
  CHECK(fs_signal.good() && fs_graph.good() && fs_adj.good());

  static std::string kHeader = "# ts qw qx qy qz x y z\n";
  static const Eigen::IOFormat CSVFormat(
      Eigen::StreamPrecision, Eigen::DontAlignCols, ", ", "\n");

  // Write the sparse graph and corresponding signal to disk.
  fs_graph << kHeader;
  for (const RepresentativeNode& node : sparse_graph_) {
    if (!node.isActive()) {
      continue;
    }

    // write graph to disk.
    fs_graph << node.getTimestampNanoseconds() << ", "
             << node.getPose().asVector().transpose().format(CSVFormat) << "\n";

    // write signal to disk.
    fs_signal << node.getResidual() << "\n";
  }

  // Write the weighted adjacency matrix to disk.
  fs_adj << adjacency_matrix_;

  fs_signal.close();
  fs_graph.close();
  fs_adj.close();
}

std::map<uint32_t, pose_graph::VertexIdList>
SparseGraph::getAllVerticesPerSubmap() const noexcept {
  if (sparse_graph_.empty()) {
    return {};
  }

  std::map<uint32_t, pose_graph::VertexIdList> all_vertices;
  for (const auto& node : sparse_graph_) {
    const uint32_t id = node.getAssociatedSubmapId();
    if (all_vertices.find(id) != all_vertices.end()) {
      continue;
    }
    const MissionGraph* mission_graph;
    if (!findMissionGraphForId(id, &mission_graph)) {
      LOG(WARNING) << "Found no mission graph for id = " << id;
      continue;
    }
    CHECK_NOTNULL(mission_graph);
    all_vertices[id] = mission_graph->getVerticesForId(id);
  }
  return all_vertices;
}

pose_graph::VertexIdList SparseGraph::getVerticesForSubmap(
    const uint32_t submap_id) const noexcept {
  if (sparse_graph_.empty()) {
    return pose_graph::VertexIdList();
  }
  std::map<uint32_t, pose_graph::VertexIdList> id_vertex_map =
      getAllVerticesPerSubmap();
  if (id_vertex_map.find(submap_id) == id_vertex_map.cend()) {
    return pose_graph::VertexIdList();
  }
  return id_vertex_map[submap_id];
}

pose_graph::VertexIdList SparseGraph::getAllMissionVertices() const noexcept {
  if (sparse_graph_.empty()) {
    return {};
  }

  pose_graph::VertexIdList all_vertices;
  for (const auto& node : sparse_graph_) {
    const uint32_t id = node.getAssociatedSubmapId();
    const MissionGraph* mission_graph;
    if (!findMissionGraphForId(id, &mission_graph)) {
      LOG(WARNING) << "Found no mission graph for id = " << id;
      continue;
    }
    CHECK_NOTNULL(mission_graph);
    const pose_graph::VertexIdList& mission_vertices =
        mission_graph->getVerticesForId(id);
    all_vertices.insert(
        all_vertices.end(), mission_vertices.cbegin(), mission_vertices.cend());
  }
  return all_vertices;
}

void SparseGraph::attachResiduals(std::map<uint32_t, double>&& residuals) {
  for (RepresentativeNode& node : sparse_graph_) {
    const uint32_t id = node.getAssociatedSubmapId();
    if (residuals.find(id) == residuals.end()) {
      // LOG(ERROR) << "Sparse graph contains different submap IDs!";
      continue;
    }

    node.setResidual(residuals[id]);
  }
}

std::string SparseGraph::getKeyForSubmapId(const uint32_t submap_id) const {
  for (const auto& name_and_mission_graph : mission_graphs_) {
    if (name_and_mission_graph.second.containsSubmap(submap_id)) {
      return name_and_mission_graph.first;
    }
  }
  return "";
}

void SparseGraph::publishGraphForBuilding() const {
  maplab_msgs::Graph graph_msg;
  ros::Time ts_ros;
  for (const RepresentativeNode& node : sparse_graph_) {
    if (!node.isActive()) {
      continue;
    }
    ts_ros = Utils::CreateRosTimestamp(node.getTimestampNanoseconds());
    const aslam::Transformation pose = node.getPose();
    const Eigen::Vector3d position = pose.getPosition();
    const uint32_t submap_id = node.getAssociatedSubmapId();

    geometry_msgs::Point pose_msg;
    pose_msg.x = position[0];
    pose_msg.y = position[1];
    pose_msg.z = position[2];

    graph_msg.submap_indices.emplace_back(submap_id);

    graph_msg.coords.emplace_back(pose_msg);
  }
  const uint32_t rows = adjacency_matrix_.rows();
  const uint32_t cols = adjacency_matrix_.cols();
  for (uint32_t i = 0u; i < rows; ++i) {
    for (uint32_t j = 0u; j < cols; ++j) {
      graph_msg.adjacency_matrix.emplace_back(adjacency_matrix_(i, j));
    }
  }

  graph_msg.header.stamp = ts_ros;
  graph_msg.header.seq = pub_seq_;
  visualization::RVizVisualizationSink::publish(
      "sparse_graph/graph", graph_msg);
}

void SparseGraph::publishTrajecotryForEvaluation() const {
  const std::string& mission_frame = "darpa";
  maplab_msgs::Trajectory traj_msg;
  ros::Time ts_ros;
  for (const RepresentativeNode& node : sparse_graph_) {
    if (!node.isActive()) {
      continue;
    }
    const uint32_t submap_id = node.getAssociatedSubmapId();
    const std::string name = getKeyForSubmapId(submap_id);
    const double residual = node.getResidual();
    const aslam::Transformation pose = node.getPose();
    ts_ros = Utils::CreateRosTimestamp(node.getTimestampNanoseconds());

    geometry_msgs::PoseStamped pose_msg;
    tf::poseStampedKindrToMsg(node.getPose(), ts_ros, mission_frame, &pose_msg);

    maplab_msgs::TrajectoryNode node_msg;
    node_msg.id = submap_id;
    node_msg.robot_name = name;
    node_msg.signal = residual;
    node_msg.pose = pose_msg;
    traj_msg.nodes.emplace_back(node_msg);
  }

  traj_msg.header.stamp = ts_ros;
  traj_msg.header.seq = pub_seq_;
  visualization::RVizVisualizationSink::publish(
      "sparse_graph/trajectory", traj_msg);
}

void SparseGraph::publishGraphForVisualization() const {
  const std::string& mission_frame = "darpa";
  nav_msgs::Path graph_msg;
  graph_msg.header.frame_id = mission_frame;

  ros::Time ts_ros;
  for (const RepresentativeNode& node : sparse_graph_) {
    if (!node.isActive()) {
      continue;
    }
    ts_ros = Utils::CreateRosTimestamp(node.getTimestampNanoseconds());
    geometry_msgs::PoseStamped node_msg;
    tf::poseStampedKindrToMsg(node.getPose(), ts_ros, mission_frame, &node_msg);
    node_msg.header.seq = node.getAssociatedSubmapId();
    graph_msg.poses.emplace_back(node_msg);
  }
  graph_msg.header.stamp = ts_ros;
  graph_msg.header.seq = pub_seq_;
  visualization::RVizVisualizationSink::publish(
      "sparse_graph/viz_graph", graph_msg);
}

void SparseGraph::publishNewSubmaps(const vi_map::VIMap* map) {
  CHECK_NOTNULL(map);
  if (sparse_graph_.empty()) {
    return;
  }
  // Iterate over all missions.
  for (const auto& mission_graph : mission_graphs_) {
    const std::string& robot_name = mission_graph.first;
    const MissionGraph& mission = mission_graph.second;
    const std::vector<uint32_t> submap_ids = mission.getAllSubmapIds();
    // Iterate over all submap ids within a mission
    for (const uint32_t submap_id : submap_ids) {
      if (wasSubmapPublished(submap_id)) {
        continue;
      }
      if (publishSubmap(map, submap_id, mission, robot_name)) {
        pub_submap_ids.emplace_back(submap_id);
      }
    }
  }
}

bool SparseGraph::publishSubmap(
    const vi_map::VIMap* map, const uint32_t submap_id,
    const MissionGraph& mission, const std::string& robot_name) const {
  CHECK_NOTNULL(map);
  if (submap_id >= submap_id_) {
    LOG(ERROR) << "Trying to publish a submap that doesn't exist (" << submap_id
               << " vs. " << std::to_string(submap_id_) << ").";
    return false;
  }
  pose_graph::VertexIdList vertex_ids = mission.getVerticesForId(submap_id);
  DenseMapBuilder map_builder(map);
  std::vector<maplab_msgs::DenseNode> dense_nodes =
      map_builder.buildMapFromVertices(vertex_ids);
  maplab_msgs::Submap submap_msg;
  submap_msg.header.seq = pub_seq_;
  submap_msg.header.stamp = ros::Time::now();
  submap_msg.robot_name = robot_name;
  submap_msg.id = submap_id;
  submap_msg.nodes = dense_nodes;

  visualization::RVizVisualizationSink::publish(
      "sparse_graph/submap", submap_msg);

  return true;
}

bool SparseGraph::wasSubmapPublished(const uint32_t submap_id) const {
  const auto it =
      std::find(pub_submap_ids.cbegin(), pub_submap_ids.cend(), submap_id);
  return it != pub_submap_ids.cend();
}

}  // namespace spg
