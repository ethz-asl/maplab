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
  CHECK_NOTNULL(map_);

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

  const std::vector<int64_t> all_ts_ns = getTimestampsFromVertices(vertices);
  depth_integration::ResourceSelectionFunction get_resources_at_ts =
      [&all_ts_ns](
          const int64_t ts_ns, const aslam::Transformation &
          /*T_G_S*/) -> bool {
    const auto it = std::find(all_ts_ns.cbegin(), all_ts_ns.cend(), ts_ns);
    return it != all_ts_ns.cend();
  };

  const vi_map::MissionIdList mission_ids = getMissionIdsFromVertices(vertices);
  depth_integration::integrateAllDepthResourcesOfType(
      mission_ids, point_cloud_resource_type_,
      false /*use_undistorted_camera_for_depth_maps*/, *map_,
      integration_function, get_resources_at_ts);

  return convertToDenseMessages(poses, clouds);
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
