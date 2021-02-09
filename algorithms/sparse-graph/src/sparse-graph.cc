#include "sparse-graph/sparse-graph.h"

#include <fstream>

#include <geometry_msgs/PoseStamped.h>
#include <glog/logging.h>
#include <nav_msgs/Path.h>

#include <minkindr_conversions/kindr_msg.h>
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
