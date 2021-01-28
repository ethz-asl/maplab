#include "sparse-graph/partitioners/avg-partitioner.h"

#include <Eigen/Dense>
#include <Eigen/Eigenvalues>
#include <Eigen/Geometry>

namespace spg {

AvgPartitioner::AvgPartitioner(const vi_map::VIMap& map)
    : BasePartitioner(map) {}

RepresentativeNode AvgPartitioner::getRepresentativesForSubmap(
    const pose_graph::VertexIdList& vertices) {
  const std::size_t n_vertices = vertices.size();
  Eigen::MatrixXd average_position(3u, 1);
  Eigen::MatrixXd average_quaternion(4u, 4u);

  for (std::size_t i = 0u; i < n_vertices; ++i) {
    const vi_map::Vertex& vertex = map_.getVertex(vertices[i]);

    // Accumulate position.
    average_position += vertex.get_p_M_I();

    // Accumulate rotations.
    Eigen::Vector4d q = vertex.get_T_M_I().getRotation().vector();
    average_quaternion += q.transpose() * q;
  }

  // Average position.
  average_position /= n_vertices;

  // Average rotation.
  average_quaternion /= n_vertices;
  Eigen::EigenSolver<Eigen::Matrix<double, 4, 4>> solver(average_quaternion);
  Eigen::Matrix<std::complex<double>, 4, 1> mat(solver.eigenvalues());
  int index;
  mat.real().maxCoeff(&index);
  Eigen::Matrix<double, 4, 1> largest_ev(
      solver.eigenvectors().real().block(0, index, 4, 1));
  Eigen::Quaterniond avg_q(
      largest_ev(0), largest_ev(1), largest_ev(2), largest_ev(3));

  // Return averaged transformation with the used vertices.
  aslam::Transformation averaged_T(avg_q, average_position);
  return RepresentativeNode(averaged_T, vertices);
}

}  // namespace spg
