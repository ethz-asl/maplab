#include "sparse-graph/sparse-graph.h"

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
    sparse_graph_.insert(mission.begin(), mission.end());
  }
}

void SparseGraph::publishLatestGraph() {
  const std::string& mission_frame = "darpa";
  nav_msgs::Path graph_msg;
  graph_msg.header.frame_id = mission_frame;

  ros::Time ts_ros;
  for (const RepresentativeNode& node : sparse_graph_) {
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

}  // namespace spg
