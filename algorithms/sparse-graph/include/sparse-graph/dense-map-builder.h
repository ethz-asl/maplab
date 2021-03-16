#ifndef SPARSE_GRAPH_DENSE_MAP_BUILDER_H_
#define SPARSE_GRAPH_DENSE_MAP_BUILDER_H_

#include <vi-map/vi-map.h>

#include <maplab_msgs/DenseNode.h>

namespace spg {

class DenseMapBuilder {
 public:
  explicit DenseMapBuilder(const vi_map::VIMap* map);
  std::vector<maplab_msgs::DenseNode> buildMapFromVertices(
      const pose_graph::VertexIdList& vertices);

 private:
  std::vector<int64_t> getTimestampsFromVertices(
      const pose_graph::VertexIdList& vertices);
  vi_map::MissionIdList getMissionIdsFromVertices(
      const pose_graph::VertexIdList& vertices);

  std::vector<maplab_msgs::DenseNode> convertToDenseMessages(
      const std::vector<geometry_msgs::PoseStamped>& poses,
      const std::vector<sensor_msgs::PointCloud2>& clouds);

  maplab_msgs::DenseNode createDenseNode(
      const geometry_msgs::PoseStamped& pose,
      const sensor_msgs::PointCloud2& cloud) const;

  const vi_map::VIMap* map_;
  const backend::ResourceType point_cloud_resource_type_;
};

}  // namespace spg

#endif  // SPARSE_GRAPH_DENSE_MAP_BUILDER_H_
