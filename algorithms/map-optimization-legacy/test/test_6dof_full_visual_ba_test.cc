#include <maplab-common/test/testing-entrypoint.h>
#include <maplab-common/test/testing-predicates.h>

#include "map-optimization-legacy/test/6dof-pose-graph-gen.h"

namespace map_optimization_legacy {

class ViwlsGraph : public ::testing::Test {
 protected:
  ViwlsGraph() {}
  virtual ~ViwlsGraph() {}

  SixDofPoseGraphGenerator graph_gen_;
};

TEST_F(ViwlsGraph, GeneratedTrajectoryFullVisualBundleAdj) {
  graph_gen_.constructCamera();

  graph_gen_.settings_.num_of_landmarks = 500;
  graph_gen_.generatePathAndLandmarks();

  const int num_of_vertices = 20;
  const int num_of_fixed_vertices = 2;
  const Eigen::Vector3d gyro_bias = Eigen::Vector3d::Zero();
  const Eigen::Vector3d accel_bias = Eigen::Vector3d::Zero();
  const double initial_position_sigma = 0.3;
  const double initial_rotation_sigma = 0.05;
  graph_gen_.fillPosegraph(
      num_of_vertices, initial_position_sigma, initial_rotation_sigma,
      gyro_bias, accel_bias, num_of_fixed_vertices);

  pose_graph::VertexIdList all_vertex_ids;
  pose_graph::EdgeIdList all_edge_ids;
  graph_gen_.posegraph_.getAllVertexIds(&all_vertex_ids);
  graph_gen_.posegraph_.getAllEdgeIds(&all_edge_ids);
  EXPECT_EQ(num_of_vertices, static_cast<int>(all_vertex_ids.size()));
  EXPECT_EQ(num_of_vertices - 1, static_cast<int>(all_edge_ids.size()));

  LOG(INFO) << "Solving...";
  graph_gen_.addVisualResidualBlocks(true, false);
  // Fix the first two vertices. We need to do it to fix the scale of
  // the problem.
  graph_gen_.fixFirstVertices(num_of_fixed_vertices, false);
  graph_gen_.corruptLandmarkPositions(0.1);
  const int max_num_of_iters = 20;
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

  typedef std::unordered_map<vi_map::LandmarkId,
                             vi_map::Landmark::Ptr>::iterator it_type;
  for (it_type iterator = graph_gen_.landmarks_.begin();
       iterator != graph_gen_.landmarks_.end(); ++iterator) {
    vi_map::Landmark::Ptr landmark_ptr = iterator->second;

    AlignedUnorderedMap<vi_map::LandmarkId, Eigen::Vector3d>::iterator
        it_landmark_pos;
    it_landmark_pos =
        graph_gen_.true_landmark_positions_.find(landmark_ptr->id());

    // Verify landmark position only for the ones that were observed at least
    // in two keyframes.
    if (graph_gen_.landmark_observation_count_.find(landmark_ptr->id())
            ->second >= 2) {
      EXPECT_NEAR_EIGEN(landmark_ptr->get_p_B(), it_landmark_pos->second, 1e-6);
    }
  }
}

}  // namespace map_optimization_legacy

MAPLAB_UNITTEST_ENTRYPOINT
