#include "sparse-graph/partitioners/avg-partitioner.h"

#include <Eigen/Dense>
#include <Eigen/Eigenvalues>
#include <Eigen/Geometry>

#include <glog/logging.h>

namespace spg {

AvgPartitioner::AvgPartitioner(const vi_map::VIMap& map)
    : BasePartitioner(map) {}

RepresentativeNodeVector AvgPartitioner::getRepresentativesForSubmap(
    const pose_graph::VertexIdList& vertices, const uint64_t submap_id) {
  const std::size_t n_vertices = vertices.size();
  if (n_vertices == 0) {
    return {};
  }

  const vi_map::Vertex& vertex = map_.getVertex(vertices[0]);
  Eigen::MatrixXd average_position = vertex.get_p_M_I();
  Eigen::Quaterniond average_quaternion =
      vertex.get_T_M_I().getRotation().toImplementation();
  int64_t average_timestamp = 0u;
  for (std::size_t i = 1u; i < n_vertices; ++i) {
    const vi_map::Vertex& vertex = map_.getVertex(vertices[i]);
    const int64_t ts_vertex_ns =
        vertex.getVisualNFrame().getMinTimestampNanoseconds();

    average_position += vertex.get_p_M_I();
    average_timestamp += ts_vertex_ns;

    // This only works well if the rotations are relatively close to each other.
    Eigen::Quaterniond q = vertex.get_T_M_I().getRotation().toImplementation();
    const float weight = 1.0f / static_cast<float>(i + 1);
    average_quaternion = average_quaternion.slerp(weight, q);
  }
  average_position /= n_vertices;
  average_timestamp /= n_vertices;

  // Return averaged transformation with the used vertices.
  aslam::Transformation averaged_T(average_quaternion, average_position);
  return {RepresentativeNode(averaged_T, average_timestamp, submap_id)};
}

}  // namespace spg
