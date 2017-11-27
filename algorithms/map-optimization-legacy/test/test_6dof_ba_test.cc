#include <maplab-common/test/testing-entrypoint.h>
#include <maplab-common/test/testing-predicates.h>

#include "map-optimization-legacy/test/6dof-pose-graph-gen.h"
#include "map-optimization-legacy/test/vi-optimization-test-helpers.h"

namespace map_optimization_legacy {

class ViwlsGraph : public ::testing::Test {
 protected:
  ViwlsGraph() {}
  virtual ~ViwlsGraph() {}
  SixDofPoseGraphGenerator graph_gen_;
};

TEST_F(ViwlsGraph, GeneratedTrajectoryInertialBundleAdj) {
  graph_gen_.constructCamera();
  graph_gen_.generatePathAndLandmarks();

  const int num_of_vertices = 20;
  const int num_of_no_noise_vertices = 1;
  const Eigen::Vector3d gyro_bias = Eigen::Vector3d::Zero();
  const Eigen::Vector3d accel_bias = Eigen::Vector3d::Zero();
  const double initial_position_sigma = 1e-4;
  const double initial_rotation_sigma = 1e-4;
  graph_gen_.fillPosegraph(
      num_of_vertices, initial_position_sigma, initial_rotation_sigma,
      gyro_bias, accel_bias, num_of_no_noise_vertices);

  pose_graph::VertexIdList all_vertex_ids;
  pose_graph::EdgeIdList all_edge_ids;
  graph_gen_.posegraph_.getAllVertexIds(&all_vertex_ids);
  graph_gen_.posegraph_.getAllEdgeIds(&all_edge_ids);
  EXPECT_EQ(num_of_vertices, static_cast<int>(all_vertex_ids.size()));
  EXPECT_EQ(num_of_vertices - 1, static_cast<int>(all_edge_ids.size()));

  LOG(INFO) << "Solving...";
  graph_gen_.addInertialResidualBlocks(true, true);
  const int max_num_of_iters = 25;
  graph_gen_.solve(max_num_of_iters);

  for (const pose_graph::VertexId& vertex_id : all_vertex_ids) {
    const vi_map::Vertex* ba_vertex = dynamic_cast<const vi_map::Vertex*>(
        graph_gen_.posegraph_.getVertexPtr(vertex_id));
    CHECK(ba_vertex) << "Couldn't cast to BA vertex type.";

    AlignedUnorderedMap<pose_graph::VertexId, Eigen::Vector3d>::iterator it_pos;
    AlignedUnorderedMap<pose_graph::VertexId, Eigen::Quaterniond>::iterator
        it_rot;
    it_pos = graph_gen_.true_vertex_positions_.find(vertex_id);
    it_rot = graph_gen_.true_vertex_rotations_.find(vertex_id);

    EXPECT_NEAR_EIGEN(it_pos->second, ba_vertex->get_p_M_I(), 1e-6);
    EXPECT_NEAR_EIGEN_QUATERNION(it_rot->second, ba_vertex->get_q_M_I(), 1e-8);
  }
}

TEST_F(ViwlsGraph, GeneratedTrajectoryVisualBundleAdj) {
  graph_gen_.constructCamera();
  graph_gen_.generatePathAndLandmarks();

  const int num_of_vertices = 20;
  const int num_of_no_noise_vertices = 1;
  const Eigen::Vector3d gyro_bias = Eigen::Vector3d::Zero();
  const Eigen::Vector3d accel_bias = Eigen::Vector3d::Zero();
  const double initial_position_sigma = 0.3;
  const double initial_rotation_sigma = 0.05;
  graph_gen_.fillPosegraph(
      num_of_vertices, initial_position_sigma, initial_rotation_sigma,
      gyro_bias, accel_bias, num_of_no_noise_vertices);

  pose_graph::VertexIdList all_vertex_ids;
  pose_graph::EdgeIdList all_edge_ids;
  graph_gen_.posegraph_.getAllVertexIds(&all_vertex_ids);
  graph_gen_.posegraph_.getAllEdgeIds(&all_edge_ids);
  EXPECT_EQ(num_of_vertices, static_cast<int>(all_vertex_ids.size()));
  EXPECT_EQ(num_of_vertices - 1, static_cast<int>(all_edge_ids.size()));

  LOG(INFO) << "Solving...";

  graph_gen_.addVisualResidualBlocks(true, true);
  const int max_num_of_iters = 25;
  graph_gen_.solve(max_num_of_iters);

  for (const pose_graph::VertexId& vertex_id : all_vertex_ids) {
    const vi_map::Vertex* ba_vertex = dynamic_cast<const vi_map::Vertex*>(
        graph_gen_.posegraph_.getVertexPtr(vertex_id));
    CHECK(ba_vertex) << "Couldn't cast to BA vertex type.";

    AlignedUnorderedMap<pose_graph::VertexId, Eigen::Vector3d>::iterator it_pos;
    AlignedUnorderedMap<pose_graph::VertexId, Eigen::Quaterniond>::iterator
        it_rot;
    it_pos = graph_gen_.true_vertex_positions_.find(vertex_id);
    it_rot = graph_gen_.true_vertex_rotations_.find(vertex_id);

    EXPECT_NEAR_EIGEN(it_pos->second, ba_vertex->get_p_M_I(), 1e-6);
    EXPECT_NEAR_EIGEN_QUATERNION(it_rot->second, ba_vertex->get_q_M_I(), 1e-8);
  }
}

TEST_F(ViwlsGraph, GeneratedTrajectoryVisualInertialBundleAdj) {
  graph_gen_.constructCamera();

  setImuSigmasZero(&graph_gen_.settings_.imu_sigmas);
  graph_gen_.generatePathAndLandmarks();

  const int num_of_vertices = 20;
  const int num_of_no_noise_vertices = 1;
  const Eigen::Vector3d gyro_bias = Eigen::Vector3d(0.1, 0, 0);
  const Eigen::Vector3d accel_bias = Eigen::Vector3d(0, 0.2, 0);
  const double initial_position_sigma = 1e-4;
  const double initial_rotation_sigma = 1e-4;

  graph_gen_.fillPosegraph(
      num_of_vertices, initial_position_sigma, initial_rotation_sigma,
      gyro_bias, accel_bias, num_of_no_noise_vertices);

  pose_graph::VertexIdList all_vertex_ids;
  pose_graph::EdgeIdList all_edge_ids;
  graph_gen_.posegraph_.getAllVertexIds(&all_vertex_ids);
  graph_gen_.posegraph_.getAllEdgeIds(&all_edge_ids);
  EXPECT_EQ(num_of_vertices, static_cast<int>(all_vertex_ids.size()));
  EXPECT_EQ(num_of_vertices - 1, static_cast<int>(all_edge_ids.size()));

  bool fix_intrinsics = false;
  bool fix_landmark_positions = true;

  Eigen::VectorXd intrinsics =
      graph_gen_.cameras_
          ->getCamera(SixDofPoseGraphGenerator::kVisualFrameIndex)
          .getParameters();
  intrinsics[0] = 110;
  intrinsics[1] = 97;
  intrinsics[2] = graph_gen_.res_u_ / 2.0 + 7;
  intrinsics[3] = graph_gen_.res_v_ / 2.0 - 4;
  graph_gen_.cameras_
      ->getCameraMutable(SixDofPoseGraphGenerator::kVisualFrameIndex)
      .setParameters(intrinsics);

  LOG(INFO) << "Solving...";
  graph_gen_.addVisualResidualBlocks(fix_intrinsics, fix_landmark_positions);
  graph_gen_.addInertialResidualBlocks(false, false);
  LOG(INFO) << "Number of parameter blocks: "
            << graph_gen_.problem_.NumResidualBlocks();
  const int max_num_of_iters = 25;
  graph_gen_.solve(max_num_of_iters);

  for (const pose_graph::VertexId& vertex_id : all_vertex_ids) {
    vi_map::Vertex* ba_vertex = dynamic_cast<vi_map::Vertex*>(
        graph_gen_.posegraph_.getVertexPtrMutable(vertex_id));
    CHECK(ba_vertex) << "Couldn't cast to BA vertex type.";

    AlignedUnorderedMap<pose_graph::VertexId, Eigen::Vector3d>::iterator it_pos;
    AlignedUnorderedMap<pose_graph::VertexId, Eigen::Quaterniond>::iterator
        it_rot;
    it_pos = graph_gen_.true_vertex_positions_.find(vertex_id);
    it_rot = graph_gen_.true_vertex_rotations_.find(vertex_id);

    EXPECT_NEAR_EIGEN(it_pos->second, ba_vertex->get_p_M_I(), 1e-6);
    EXPECT_NEAR_EIGEN_QUATERNION(it_rot->second, ba_vertex->get_q_M_I(), 1e-8);
    Eigen::Map<Eigen::Vector3d> bw(ba_vertex->getGyroBiasMutable());
    EXPECT_NEAR_EIGEN(bw, gyro_bias, 1e-8);

    Eigen::Map<Eigen::Vector3d> ba(ba_vertex->getAccelBiasMutable());
    EXPECT_NEAR_EIGEN(ba, accel_bias, 1e-8);
  }

  EXPECT_NEAR_EIGEN(
      graph_gen_.cameras_
          ->getCamera(SixDofPoseGraphGenerator::kVisualFrameIndex)
          .getParameters(),
      Eigen::Vector4d(
          graph_gen_.fu_, graph_gen_.fv_, graph_gen_.cu_, graph_gen_.cv_),
      1e-8);
}

}  // namespace map_optimization_legacy

MAPLAB_UNITTEST_ENTRYPOINT
