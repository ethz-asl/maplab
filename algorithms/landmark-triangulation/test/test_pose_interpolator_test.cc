#include <memory>
#include <random>
#include <unordered_map>
#include <unordered_set>

#include <aslam/cameras/camera-pinhole.h>
#include <aslam/cameras/distortion-fisheye.h>
#include <aslam/cameras/ncamera.h>
#include <aslam/frames/visual-frame.h>
#include <glog/logging.h>
#include <map-optimization-legacy/test/6dof-vi-map-gen.h>
#include <maplab-common/test/testing-entrypoint.h>
#include <maplab-common/test/testing-predicates.h>
#include <maplab-common/unique-id.h>
#include <vi-map/pose-graph.h>
#include <vi-map/vi-map.h>

#include "landmark-triangulation/pose-interpolator.h"

namespace landmark_triangulation {

class ViwlsGraph : public ::testing::Test {
 protected:
  virtual ~ViwlsGraph() {}

 public:
  map_optimization_legacy::SixDofVIMapGenerator vimap_gen_;
};

TEST_F(ViwlsGraph, PoseInterpolationTestAtVertices) {
  vimap_gen_.generateVIMap();
  vi_map::VIMap& vi_map = vimap_gen_.vi_map_;

  vi_map::MissionIdList mission_ids;
  vi_map.getAllMissionIds(&mission_ids);
  CHECK_EQ(mission_ids.size(), 1u);
  vi_map::MissionId mission_id = mission_ids[0];

  pose_graph::VertexIdList all_vertices;
  vi_map.getAllVertexIdsInMissionAlongGraph(mission_id, &all_vertices);

  Eigen::Matrix<int64_t, 1, Eigen::Dynamic> imu_timestamps;
  imu_timestamps.resize(Eigen::NoChange, all_vertices.size());

  int index = 0;
  pose_graph::VertexIdList requested_vertices;
  for (const pose_graph::VertexId& vertex_id : all_vertices) {
    const vi_map::Vertex& vertex = vi_map.getVertex(vertex_id);
    pose_graph::EdgeIdSet outgoing_edges;
    vertex.getOutgoingEdges(&outgoing_edges);
    pose_graph::EdgeId outgoing_imu_edge_id;
    for (const pose_graph::EdgeId& edge_id : outgoing_edges) {
      if (vi_map.getEdgeType(edge_id) == pose_graph::Edge::EdgeType::kViwls) {
        outgoing_imu_edge_id = edge_id;
        break;
      }
    }
    if (outgoing_imu_edge_id.isValid()) {
      const vi_map::ViwlsEdge& imu_edge =
          vi_map.getEdgeAs<vi_map::ViwlsEdge>(outgoing_imu_edge_id);
      const Eigen::Matrix<int64_t, 1, Eigen::Dynamic>& timestamps =
          imu_edge.getImuTimestamps();
      if (timestamps.cols() > 0) {
        imu_timestamps(0, index) = timestamps(0, 0);
        ++index;
        requested_vertices.push_back(vertex_id);
      }
    }
  }
  imu_timestamps.conservativeResize(Eigen::NoChange, index);

  ASSERT_FALSE(requested_vertices.empty());

  PoseInterpolator pose_interpolator;
  aslam::TransformationVector T_M_I_list;
  pose_interpolator.getPosesAtTime(
      vi_map, mission_id, imu_timestamps, &T_M_I_list);

  ASSERT_EQ(static_cast<int>(T_M_I_list.size()), imu_timestamps.cols());

  for (int i = 0; i < imu_timestamps.cols(); ++i) {
    const aslam::Transformation& T_MI = T_M_I_list[i];
    const vi_map::Vertex& vertex = vi_map.getVertex(requested_vertices[i]);
    EXPECT_NEAR_ASLAM_TRANSFORMATION(T_MI, vertex.get_T_M_I(), 1e-5);
  }
}

TEST_F(ViwlsGraph, PoseInterpolationTestBetweenVertices) {
  vimap_gen_.generateVIMap();
  vi_map::VIMap& vi_map = vimap_gen_.vi_map_;
  map_optimization_legacy::SixDofPoseGraphGenerator& graph_generator =
      vimap_gen_.graph_gen_;

  Eigen::VectorXd imu_timestamps_seconds =
      graph_generator.imu_timestamps_seconds_;

  ASSERT_NE(imu_timestamps_seconds.rows(), 0);

  // Only use half of the data.
  imu_timestamps_seconds.conservativeResize(
      imu_timestamps_seconds.rows() / 2, Eigen::NoChange);

  vi_map::MissionIdList mission_ids;
  vi_map.getAllMissionIds(&mission_ids);
  CHECK_EQ(mission_ids.size(), 1u);
  vi_map::MissionId mission_id = mission_ids[0];

  Eigen::Matrix<int64_t, 1, Eigen::Dynamic> imu_timestamps;
  imu_timestamps.resize(Eigen::NoChange, imu_timestamps_seconds.rows());

  constexpr double kSecondsToNanoSeconds = 1e9;
  for (int i = 0; i < imu_timestamps_seconds.rows(); ++i) {
    imu_timestamps(0, i) = kSecondsToNanoSeconds * imu_timestamps_seconds(i, 0);
  }

  PoseInterpolator pose_interpolator;
  aslam::TransformationVector T_M_I_list;
  pose_interpolator.getPosesAtTime(
      vi_map, mission_id, imu_timestamps, &T_M_I_list);

  ASSERT_EQ(static_cast<int>(T_M_I_list.size()), imu_timestamps.cols());

  // Get the ground-truth values from the simulation.
  const Eigen::Matrix3Xd& p_GI_gt = graph_generator.positions_;
  const Eigen::Matrix4Xd& q_GI_gt = graph_generator.rotations_;

  ASSERT_EQ(p_GI_gt.cols() / 2, imu_timestamps.cols());
  ASSERT_EQ(q_GI_gt.cols() / 2, imu_timestamps.cols());

  for (int i = 0; i < imu_timestamps.cols(); ++i) {
    const aslam::Transformation& T_MI = T_M_I_list[i];
    Eigen::Quaterniond q_GI;
    q_GI.coeffs() = q_GI_gt.col(i);
    Eigen::Matrix<double, 3, 1> p_GI = p_GI_gt.col(i);

    // TODO(slynen): Change to expect.
    ASSERT_NEAR_EIGEN(T_MI.getPosition(), p_GI, 1e-4);
    ASSERT_NEAR_EIGEN_QUATERNION(
        T_MI.getRotation().toImplementation(), q_GI, 1e-4);
  }
}

}  // namespace landmark_triangulation

MAPLAB_UNITTEST_ENTRYPOINT
