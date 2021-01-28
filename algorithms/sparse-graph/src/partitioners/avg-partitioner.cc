#include "sparse-graph/partitioners/avg-partitioner.h"

#include <Eigen/Dense>
#include <Eigen/Eigenvalues>
#include <Eigen/Geometry>

#include <glog/logging.h>

namespace spg {

AvgPartitioner::AvgPartitioner(const vi_map::VIMap& map)
    : BasePartitioner(map) {}

RepresentativeNode AvgPartitioner::getRepresentativesForSubmap(
    const pose_graph::VertexIdList& vertices) {
  const std::size_t n_vertices = vertices.size();
  if (n_vertices == 0) {
    return RepresentativeNode();
  }

  const vi_map::Vertex& vertex = map_.getVertex(vertices[0]);
  Eigen::MatrixXd average_position = vertex.get_p_M_I();
  Eigen::Quaterniond average_quaternion =
      vertex.get_T_M_I().getRotation().toImplementation();
  for (std::size_t i = 1u; i < n_vertices; ++i) {
    const vi_map::Vertex& vertex = map_.getVertex(vertices[i]);

    average_position += vertex.get_p_M_I();

    Eigen::Quaterniond q = vertex.get_T_M_I().getRotation().toImplementation();
    const float weight = 1.0f / static_cast<float>(i + 1);
    average_quaternion = average_quaternion.slerp(weight, q);
  }
  average_position /= n_vertices;

  // Return averaged transformation with the used vertices.
  aslam::Transformation averaged_T(average_quaternion, average_position);
  return RepresentativeNode(averaged_T, vertices);
}

}  // namespace spg
