#include <memory>
#include <vector>

#include <Eigen/Core>
#include <glog/logging.h>
#include <gtest/gtest.h>

#include <aslam/cameras/camera-pinhole.h>
#include <aslam/cameras/distortion-fisheye.h>
#include <aslam/common/memory.h>
#include <aslam/frames/visual-frame.h>
#include <maplab-common/pose_types.h>
#include <maplab-common/test/testing-entrypoint.h>
#include <maplab-common/test/testing-predicates.h>
#include <vi-map/landmark.h>
#include <vi-map/mission.h>
#include <vi-map/pose-graph.h>
#include <vi-map/vertex.h>
#include <vi-map/vi_map.pb.h>
#include <vi-map/viwls-edge.h>

namespace map_optimization_legacy {

class ViwlsGraph : public ::testing::Test {
 protected:
  typedef aslam::FisheyeDistortion DistortionType;
  typedef aslam::PinholeCamera CameraType;

  virtual void SetUp() {
    cameras_ = aslam::NCamera::createTestNCamera(kNumCameras);
  }

  aslam::NCamera::Ptr cameras_;
  static constexpr int kNumCameras = 1;
  static constexpr int kVisualFrameIndex = kNumCameras - 1;
};

TEST_F(ViwlsGraph, SimpleTest) {
  vi_map::PoseGraph posegraph;
  Aligned<std::vector, vi_map::VIMission> missions;
  vi_map::LandmarkIdList landmark_ids;

  vi_map::VIMission mission;
  vi_map::MissionId mission_id;
  mission_id.fromHexString("aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa0");
  missions.push_back(mission);

  aslam::VisualFrame::DescriptorsT descriptors;
  descriptors.resize(48, 1);
  descriptors.setRandom();

  vi_map::Landmark landmark;
  vi_map::LandmarkId landmark_id;
  landmark_id.fromHexString("aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa1");
  landmark.setId(landmark_id);
  landmark.set_p_B(pose::Position3D(1, 0, 0));
  landmark.set_p_B_Covariance(Eigen::Matrix<double, 3, 3>::Identity());
  landmark_ids.push_back(landmark_id);

  Eigen::Matrix<double, 6, 1> imu_bias;
  imu_bias << 1, 2, 3, 4, 5, 6;

  Eigen::Matrix<double, 2, 1> points0, points1;
  points0 << 1, 2;
  points1 << 3, 4;

  Eigen::VectorXd uncertainties0, uncertainties1;
  uncertainties0.resize(1);
  uncertainties0 << 1;
  uncertainties1.resize(1);
  uncertainties1 << 0.5;

  pose_graph::VertexId vertex0;
  pose_graph::VertexId vertex1;
  vertex0.fromHexString("aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa2");
  vertex1.fromHexString("aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa3");

  aslam::FrameId frame_id0;
  aslam::FrameId frame_id1;
  frame_id0.fromHexString("aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa4");
  frame_id1.fromHexString("aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa5");

  posegraph.addVIVertex(
      vertex0, imu_bias, points0, uncertainties0, descriptors, landmark_ids,
      mission.id(), frame_id0, 0, cameras_);
  posegraph.addVIVertex(
      vertex1, imu_bias, points1, uncertainties1, descriptors, landmark_ids,
      mission.id(), frame_id1, 1, cameras_);
  EXPECT_TRUE(posegraph.vertexExists(vertex0));
  EXPECT_TRUE(posegraph.vertexExists(vertex1));

  Eigen::Matrix<double, 6, 1> imu_data;
  imu_data.setOnes();

  // Assume starting vertex is at zero time and then each IMU measurement
  // should have a corresponding timestamp along the edge.
  Eigen::Matrix<int64_t, 1, 1> imu_timestamps;
  imu_timestamps << 1;

  pose_graph::EdgeId edge0;
  posegraph.addVIEdge(edge0, vertex0, vertex1, imu_timestamps, imu_data);
  EXPECT_TRUE(posegraph.edgeExists(edge0));

  pose_graph::VertexIdList all_vertex_ids;
  pose_graph::EdgeIdList all_edge_ids;
  posegraph.getAllVertexIds(&all_vertex_ids);
  posegraph.getAllEdgeIds(&all_edge_ids);
  EXPECT_EQ(1u, all_edge_ids.size());
  EXPECT_EQ(2u, all_vertex_ids.size());

  const vi_map::Vertex* ba_vertex =
      dynamic_cast<const vi_map::Vertex*>(posegraph.getVertexPtr(vertex0));
  CHECK(ba_vertex);  // Continuing without a valid ptr makes no sense.

  const Eigen::Matrix2Xd& image_points_distorted =
      ba_vertex->getVisualFrame(kVisualFrameIndex).getKeypointMeasurements();
  EXPECT_EQ(1, image_points_distorted.cols());
  EXPECT_NEAR_EIGEN(image_points_distorted, points0, 1e-15);

  const Eigen::VectorXd& image_points_uncertainties =
      ba_vertex->getVisualFrame(kVisualFrameIndex)
          .getKeypointMeasurementUncertainties();
  EXPECT_EQ(1, image_points_uncertainties.cols());
  EXPECT_NEAR_EIGEN(image_points_uncertainties, uncertainties0, 1e-15);
}

}  // namespace map_optimization_legacy

MAPLAB_UNITTEST_ENTRYPOINT
