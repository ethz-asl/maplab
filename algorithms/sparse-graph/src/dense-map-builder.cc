#include "sparse-graph/dense-map-builder.h"

#include <depth-integration/depth-integration.h>
#include <minkindr_conversions/kindr_msg.h>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <glog/logging.h>

#include "sparse-graph/common/utils.h"

namespace spg {

DenseMapBuilder::DenseMapBuilder(const vi_map::VIMap* map)
    : map_(map),
      point_cloud_resource_type_(backend::ResourceType::kPointCloudXYZI) {}

std::vector<maplab_msgs::DenseNode> DenseMapBuilder::buildMapFromVertices(
    const pose_graph::VertexIdList& vertices) {
  const std::vector<int64_t> timestamps_ns =
      getTimestampsFromVertices(vertices);
  const vi_map::MissionIdList mission_ids = getMissionIdsFromVertices(vertices);

  return buildMap(timestamps_ns, mission_ids);
}

std::vector<maplab_msgs::DenseNode> DenseMapBuilder::buildMapFromNodes(
    const std::vector<RepresentativeNode>& nodes,
    const vi_map::MissionId& mission_id) {
  if (nodes.empty()) {
    return {};
  }
  // If we don't have pointclouds associated with the nodes
  // we need to retrieve them manually.
  if (!nodes[0].hasPointCloud()) {
    LOG(ERROR) << "Node has no pointclouds!!!";
    std::vector<int64_t> timestamps_ns = getTimestampsFromNodes(nodes);
    return buildMap(timestamps_ns, {mission_id});
  }

  std::vector<geometry_msgs::PoseStamped> poses;
  std::vector<sensor_msgs::PointCloud2> clouds;
  const std::string mission_frame = "darpa";
  for (const RepresentativeNode& n : nodes) {
    const int64_t ts_ns = n.getTimestampNanoseconds();
    const ros::Time ts_ros = Utils::CreateRosTimestamp(ts_ns);
    const aslam::Transformation& T_G_N = n.getPose();  // node in global frame.
    const resources::PointCloud& pc_S = n.getPointCloud();

    geometry_msgs::PoseStamped pose_msg;
    tf::poseStampedKindrToMsg(T_G_N, ts_ros, mission_frame, &pose_msg);
    poses.emplace_back(pose_msg);

    sensor_msgs::PointCloud2 ros_point_cloud_S;
    backend::convertPointCloudType(pc_S, &ros_point_cloud_S);
    clouds.emplace_back(ros_point_cloud_S);
  }
  return convertToDenseMessages(poses, clouds);
}

std::vector<maplab_msgs::DenseNode> DenseMapBuilder::buildMap(
    const std::vector<int64_t>& timestamps_ns,
    const vi_map::MissionIdList& mission_ids) {
  CHECK_NOTNULL(map_);
  if (timestamps_ns.empty() || mission_ids.empty()) {
    return {};
  }

  // Integrator function for accumulating pointclouds and poses.
  std::vector<geometry_msgs::PoseStamped> poses;
  std::vector<sensor_msgs::PointCloud2> clouds;
  const std::string mission_frame = "darpa";
  depth_integration::IntegrationFunctionPointCloudMaplabWithTs
      integration_function = [&poses, &clouds, &mission_frame](
                                 const int64_t ts_ns,
                                 const aslam::Transformation& T_G_S,
                                 const resources::PointCloud& points_S) {
        geometry_msgs::PoseStamped pose_msg;
        const ros::Time ts_ros = Utils::CreateRosTimestamp(ts_ns);
        tf::poseStampedKindrToMsg(T_G_S, ts_ros, mission_frame, &pose_msg);
        poses.emplace_back(pose_msg);

        sensor_msgs::PointCloud2 ros_point_cloud_S;
        backend::convertPointCloudType(points_S, &ros_point_cloud_S);
        clouds.emplace_back(ros_point_cloud_S);
      };

  const int64_t eps_ns = 50000000;  // 50ms
  auto comp = [](const int64_t lhs, const int64_t rhs) {
    return std::abs(lhs - rhs) <= eps_ns;
  };

  // Filter out resources.
  depth_integration::ResourceSelectionFunction get_resources_at_ts =
      [&timestamps_ns, &comp](
          const int64_t ts_ns, const aslam::Transformation&) -> bool {
    for (const int64_t& ts : timestamps_ns) {
      if (comp(ts, ts_ns)) {
        return true;
      }
    }
    return false;
  };

  // Integrate.
  depth_integration::integrateAllDepthResourcesOfType(
      mission_ids, point_cloud_resource_type_,
      false /*use_undistorted_camera_for_depth_maps*/, *map_,
      integration_function, get_resources_at_ts);
  return convertToDenseMessages(poses, clouds);
}

std::vector<int64_t> DenseMapBuilder::getTimestampsFromNodes(
    const std::vector<RepresentativeNode>& nodes) {
  std::vector<int64_t> timestamps_ns;

  auto getter_ts = [](const RepresentativeNode& n) {
    return n.getTimestampNanoseconds();
  };
  std::transform(
      nodes.cbegin(), nodes.cend(), std::back_inserter(timestamps_ns),
      getter_ts);

  return timestamps_ns;
}

std::vector<int64_t> DenseMapBuilder::getTimestampsFromVertices(
    const pose_graph::VertexIdList& vertices) {
  CHECK_NOTNULL(map_);
  std::vector<int64_t> timestamps_ns;
  for (const pose_graph::VertexId& vertex_id : vertices) {
    if (!map_->hasVertex(vertex_id)) {
      LOG(ERROR) << "Vertex " << vertex_id << " not found.";
      continue;
    }
    const vi_map::Vertex& vertex = map_->getVertex(vertex_id);
    const int64_t timestamp_ns = vertex.getMinTimestampNanoseconds();
    timestamps_ns.emplace_back(timestamp_ns);
  }

  return timestamps_ns;
}

vi_map::MissionIdList DenseMapBuilder::getMissionIdsFromVertices(
    const pose_graph::VertexIdList& vertices) {
  CHECK_NOTNULL(map_);
  vi_map::MissionIdList mission_ids;
  for (const pose_graph::VertexId vertex_id : vertices) {
    if (!map_->hasVertex(vertex_id)) {
      LOG(ERROR) << "Vertex " << vertex_id << " not found.";
      continue;
    }
    const vi_map::Vertex& vertex = map_->getVertex(vertex_id);
    const vi_map::MissionId& mission_id = vertex.getMissionId();
    const auto it =
        std::find(mission_ids.cbegin(), mission_ids.cend(), mission_id);
    if (it == mission_ids.cend()) {
      mission_ids.emplace_back(mission_id);
    }
  }

  return mission_ids;
}

std::vector<maplab_msgs::DenseNode> DenseMapBuilder::convertToDenseMessages(
    const std::vector<geometry_msgs::PoseStamped>& poses,
    const std::vector<sensor_msgs::PointCloud2>& clouds) {
  std::vector<maplab_msgs::DenseNode> dense_nodes;
  const std::size_t n_poses = poses.size();
  if (n_poses != clouds.size()) {
    LOG(ERROR) << "Poses for dense messages are not equal "
               << "to the number of pointcloud.";
    return dense_nodes;
  }

  dense_nodes.resize(n_poses);
  for (std::size_t i = 0u; i < n_poses; ++i) {
    dense_nodes[i] = createDenseNode(poses[i], clouds[i]);
  }

  return dense_nodes;
}

maplab_msgs::DenseNode DenseMapBuilder::createDenseNode(
    const geometry_msgs::PoseStamped& pose,
    const sensor_msgs::PointCloud2& cloud) const {
  maplab_msgs::DenseNode node;
  node.pose = pose;
  node.cloud = cloud;
  return node;
}

}  // namespace spg
