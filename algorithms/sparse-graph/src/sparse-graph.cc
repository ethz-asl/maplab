#include "sparse-graph/sparse-graph.h"

#include <fstream>

#include <geometry_msgs/PoseStamped.h>
#include <glog/logging.h>
#include <nav_msgs/Path.h>

#include <minkindr_conversions/kindr_msg.h>
#include <vi-map-helpers/vi-map-nearest-neighbor-lookup.h>
#include <vi-map-helpers/vi-map-queries.h>
#include <visualization/common-rviz-visualization.h>
#include <visualization/rviz-visualization-sink.h>

namespace spg {

SparseGraph::SparseGraph() : submap_id_(0u) {}

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

void SparseGraph::publishLatestGraph() {
  const std::string& mission_frame = "darpa";
  nav_msgs::Path graph_msg;
  graph_msg.header.frame_id = mission_frame;

  ros::Time ts_ros;
  for (const RepresentativeNode& node : sparse_graph_) {
    if (!node.isActive()) {
      continue;
    }
    ts_ros = createRosTimestamp(node.getTimestampNanoseconds());
    geometry_msgs::PoseStamped node_msg;
    tf::poseStampedKindrToMsg(node.getPose(), ts_ros, mission_frame, &node_msg);
    node_msg.header.seq = node.getAssociatedSubmapId();
    graph_msg.poses.emplace_back(node_msg);
  }
  graph_msg.header.stamp = ts_ros;
  visualization::RVizVisualizationSink::publish(
      "global_sparse_graph", graph_msg);
}

void SparseGraph::publishLatestGraphWithCovs(
    const std::vector<Eigen::MatrixXd>& covs) {
  if (covs.empty()) {
    return;
  }

  VLOG(1) << "Received " << covs.size() << " covariances, last one:\n"
          << covs.back() << "\n";
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

Eigen::MatrixXd SparseGraph::computeAdjacencyMatrix(const vi_map::VIMap* map) {
  CHECK_NOTNULL(map);
  if (sparse_graph_.empty()) {
    return Eigen::MatrixXd{};
  }

  const std::size_t n_nodes = sparse_graph_.size();
  Eigen::MatrixXd weighted_adjacency = Eigen::MatrixXd::Zero(n_nodes, n_nodes);

  vi_map_helpers::VIMapNearestNeighborLookupVertexId nn_query_database(*map);
  const double search_radius = 5.0;

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
        const double w_d = computeDistanceBetweenNodes(i, j);
        const double w_c = computeCoObservability(map, i, j);

        // Set the weights for the adjacency
        // which is a symmetric and undirected adjacency matrix.
        weighted_adjacency(i, j) = w_d + w_c;
        weighted_adjacency(j, i) = weighted_adjacency(i, j);
      }
    }
  }
  return weighted_adjacency;
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
      LOG(WARNING) << "Found no vertex in mission graph for id = " << v;
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
  LOG(ERROR) << "Accumulated coobs: " << accumulated_coobs;
  const double lambda = 0.4;

  return 1 - std::pow(std::exp(-lambda), accumulated_coobs);
}

ros::Time SparseGraph::createRosTimestamp(const int64_t ts_ns) const {
  CHECK_GE(ts_ns, 0);
  static constexpr uint32_t kNanosecondsPerSecond = 1e9;
  const uint64_t timestamp_u64 = static_cast<uint64_t>(ts_ns);
  const uint32_t ros_timestamp_sec = timestamp_u64 / kNanosecondsPerSecond;
  const uint32_t ros_timestamp_nsec =
      timestamp_u64 - (ros_timestamp_sec * kNanosecondsPerSecond);
  return ros::Time(ros_timestamp_sec, ros_timestamp_nsec);
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
  std::ofstream fs_graph(graph_out_path, std::ofstream::trunc);
  std::ofstream fs_signal(signal_out_path, std::ofstream::trunc);
  CHECK(fs_signal.good() && fs_graph.good());

  static std::string kHeader = "# ts qw qx qy qz x y z\n";
  static const Eigen::IOFormat CSVFormat(
      Eigen::StreamPrecision, Eigen::DontAlignCols, ", ", "\n");

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
  fs_signal.close();
  fs_graph.close();
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
  const std::size_t n_residuals = residuals.size();
  for (RepresentativeNode& node : sparse_graph_) {
    const uint32_t id = node.getAssociatedSubmapId();
    if (residuals.find(id) == residuals.end()) {
      LOG(ERROR) << "Sparse graph contains different submap IDs!";
      continue;
    }

    node.setResidual(residuals[id]);
  }
}

}  // namespace spg
