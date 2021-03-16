#include "sparse-graph/dense-map-builder.h"

#include <depth-integration/depth-integration.h>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <glog/logging.h>

namespace spg {

DenseMapBuilder::DenseMapBuilder(const vi_map::VIMap* map) : map_(map) {}

void DenseMapBuilder::buildMapFromVertices(
    const pose_graph::VertexIdList& vertices) {
  CHECK_NOTNULL(map_);

  std::vector<geometry_msgs::PoseStamped> poses;

  depth_integration::IntegrationFunctionPointCloudMaplab integration_function =
      [&point_cloud_G](
          const aslam::Transformation& T_G_S,
          const resources::PointCloud& points_S) {
        geometry_msgs::PoseStamped pose_msg;

        tf::poseStampedKindrToMsg(T_G_S, ts_ros, mission_frame, &pose_msg);

        point_cloud_G->appendTransformed(points_S, T_G_S);
      };

  depth_integration::ResourceSelectionFunction get_resources_in_radius =
      [&radius_m, &center_G](
          const int64_t timestamp_ns,
          const aslam::Transformation& T_G_S) -> bool {
    return (T_G_S.getPosition() - center_G).norm() < radius_m;
  };

  const vi_map::MissionIdList mission_ids = getMissionIdsFromVertices(vertices);
  const std::vector<int64_t> timestamps_ns =
      getTimestampsFromVertices(vertices);
}

std::vector<int64_t> DenseMapBuilder::getTimestampsFromVertices(
    const pose_graph::VertexIdList& vertices) {
  CHECK_NOTNULL(map_);
  std::vector<int64_t> timestamps_ns;
  for (const pose_graph::VertexId vertex : vertices) {
    if (!map_.hasVertex(vertex)) {
      LOG(ERROR) << "Vertex " << vertex << " not found.";
      continue;
    }
    const vi_map::Vertex& vertex = map_.getVertex(vertices[i]);
    const int64_t timestamp_ns = vertex.getMinTimestampNanoseconds();
    timestamps_ns.emplace_back(timestamp_ns);
  }

  return timestamps_ns;
}

vi_map::MissionIdList DenseMapBuilder::getMissionIdsFromVertices(
    const pose_graph::VertexIdList& vertices) {
  CHECK_NOTNULL(map_);
  vi_map::MissionIdList mission_ids;
  for (const pose_graph::VertexId vertex : vertices) {
    if (!map_.hasVertex(vertex)) {
      LOG(ERROR) << "Vertex " << vertex << " not found.";
      continue;
    }
    const vi_map::MissionId& mission_id = vertex.getMissionId();
    const auto it =
        std::find(mission_ids.cbegin(), mission_ids.cend(), mission_id);
    if (it == mission_ids.cend()) {
      mission_ids.emplace_back(mission_id);
    }
  }

  return mission_ids;
}

}  // namespace spg
