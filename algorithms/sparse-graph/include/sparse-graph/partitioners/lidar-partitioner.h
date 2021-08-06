#ifndef SPARSE_GRAPH_PARTITIONERS_LIDAR_PARTITIONER_H_
#define SPARSE_GRAPH_PARTITIONERS_LIDAR_PARTITIONER_H_

#include "sparse-graph/partitioners/base-partitioner.h"

#include <unordered_map>

namespace spg {

class LidarPartitioner : public BasePartitioner {
 public:
  explicit LidarPartitioner(const vi_map::VIMap& map);
  virtual ~LidarPartitioner() = default;
  RepresentativeNodeVector getRepresentativesForSubmap(
      const pose_graph::VertexIdList& vertices,
      const uint64_t submap_id) override;

 private:
  void initializeSensorMapping();
  bool shouldInsertNode(
      const RepresentativeNodeVector& nodes,
      const RepresentativeNode& cur_node) const noexcept;

  const backend::ResourceType point_cloud_resource_type_;
  typedef std::unordered_map<aslam::SensorId, backend::TemporalResourceIdBuffer>
      SensorsToResourceMap;
  std::unordered_map<vi_map::MissionId, aslam::SensorId>
      mission_to_lidar_sensor_map_;
};

}  // namespace spg

#endif  // SPARSE_GRAPH_PARTITIONERS_LIDAR_PARTITIONER_H_
