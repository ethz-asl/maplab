#include <algorithm>
#include <vector>

#include <Eigen/Core>

#include <aslam/common/memory.h>
#include <maplab-common/pose_types.h>
#include <maplab-common/test/testing-entrypoint.h>
#include <maplab-common/test/testing-predicates.h>
#include <posegraph/pose-graph.h>
#include <posegraph/vertex.h>

#include "vi-map/check-map-consistency.h"
#include "vi-map/vi-map.h"

namespace vi_map {

class MapConsistencyCheckTest : public ::testing::Test {
 protected:
  MapConsistencyCheckTest() {}
  virtual void SetUp() {
    cameras_ = aslam::NCamera::createTestNCamera(kNumCameras);

    common::generateId(&mission_id1_);
    populatePosegraph(mission_id1_, &vertex_ids_mission_1_);
  }

  void populatePosegraph(
      const MissionId& mission_id,
      std::vector<pose_graph::VertexId>* vertex_ids);
  void addVisualFrameToVertex(
      const aslam::NCamera::Ptr& n_camera_ptr, vi_map::Vertex* vertex_ptr);
  void addCommonLandmarkIds(
      unsigned int count, const pose_graph::VertexId& vertex0_id,
      const pose_graph::VertexId& vertex1_id);
  void addInvalidEdge();
  void addOrphanedTrajectory();
  void addOrphanedVertex();
  void addMissionWithVisualNFrame();
  void addLandmarkAndVertexReference(
      const vi_map::LandmarkId& global_landmark_id,
      const pose_graph::VertexId& storing_vertex_id);
  void addMissionUsingAnExistingNCamera(const aslam::NCamera::Ptr& n_cameras);
  void addSecondMission();
  void addLoopClosureEdges();

  std::vector<pose_graph::VertexId> vertex_ids_mission_1_;
  std::vector<pose_graph::VertexId> vertex_ids_mission_2_;
  VIMap map_;
  vi_map::MissionId mission_id1_;
  vi_map::MissionId mission_id2_;
  aslam::NCamera::Ptr cameras_;

  static constexpr unsigned int kNumOfVertices = 4;
  static constexpr unsigned int kNumCameras = 1;
  static constexpr unsigned int kVisualFrameIndex = 0;
  static constexpr unsigned int kNumOfKeypointsPerVertex = 500;
  static constexpr int kDescriptorBytes = 48;

 private:
  void createMission(
      const pose_graph::VertexId& root_vertex_id,
      const vi_map::MissionId& mission_id,
      const aslam::NCamera::Ptr& n_camera_ptr);
};

void MapConsistencyCheckTest::addLoopClosureEdges() {
  common::generateId(&mission_id2_);
  populatePosegraph(mission_id2_, &vertex_ids_mission_2_);

  constexpr double kSwitchVariable = 1.0;
  constexpr double kSwitchVariableVariance = 1.0;
  const aslam::TransformationCovariance kT_A_B_covariance =
      aslam::TransformationCovariance::Identity();

  for (const pose_graph::VertexId& source_vertex_id_mission_1 :
       vertex_ids_mission_1_) {
    for (const pose_graph::VertexId& destination_vertex_id_mission_1 :
         vertex_ids_mission_1_) {
      if (source_vertex_id_mission_1 == destination_vertex_id_mission_1) {
        continue;
      }
      pose_graph::EdgeId edge_id;
      common::generateId(&edge_id);
      const aslam::Transformation T_G_I_source =
          map_.getVertex_T_G_I(source_vertex_id_mission_1);
      const aslam::Transformation T_G_I_dest =
          map_.getVertex_T_G_I(destination_vertex_id_mission_1);
      const aslam::Transformation T_A_B = T_G_I_source.inverse() * T_G_I_dest;

      LoopClosureEdge::UniquePtr loop_closure_edge =
          aligned_unique<LoopClosureEdge>(
              edge_id, source_vertex_id_mission_1,
              destination_vertex_id_mission_1, kSwitchVariable,
              kSwitchVariableVariance, T_A_B, kT_A_B_covariance);
      map_.addEdge(std::move(loop_closure_edge));
    }

    for (const pose_graph::VertexId& destination_vertex_id_mission_2 :
         vertex_ids_mission_2_) {
      pose_graph::EdgeId edge_id;
      common::generateId(&edge_id);
      const aslam::Transformation T_G_I_source =
          map_.getVertex_T_G_I(source_vertex_id_mission_1);
      const aslam::Transformation T_G_I_dest =
          map_.getVertex_T_G_I(destination_vertex_id_mission_2);
      const aslam::Transformation T_A_B = T_G_I_source.inverse() * T_G_I_dest;

      LoopClosureEdge::UniquePtr loop_closure_edge =
          aligned_unique<LoopClosureEdge>(
              edge_id, source_vertex_id_mission_1,
              destination_vertex_id_mission_2, kSwitchVariable,
              kSwitchVariableVariance, T_A_B, kT_A_B_covariance);
      map_.addEdge(std::move(loop_closure_edge));
    }
  }

  for (const pose_graph::VertexId& source_vertex_id_mission_2 :
       vertex_ids_mission_2_) {
    for (const pose_graph::VertexId& destination_vertex_id_mission_2 :
         vertex_ids_mission_2_) {
      if (source_vertex_id_mission_2 == destination_vertex_id_mission_2) {
        continue;
      }
      pose_graph::EdgeId edge_id;
      common::generateId(&edge_id);
      const aslam::Transformation T_G_I_source =
          map_.getVertex_T_G_I(source_vertex_id_mission_2);
      const aslam::Transformation T_G_I_dest =
          map_.getVertex_T_G_I(destination_vertex_id_mission_2);
      const aslam::Transformation T_A_B = T_G_I_source.inverse() * T_G_I_dest;

      LoopClosureEdge::UniquePtr loop_closure_edge =
          aligned_unique<LoopClosureEdge>(
              edge_id, source_vertex_id_mission_2,
              destination_vertex_id_mission_2, kSwitchVariable,
              kSwitchVariableVariance, T_A_B, kT_A_B_covariance);
      map_.addEdge(std::move(loop_closure_edge));
    }

    for (const pose_graph::VertexId& destination_vertex_id_mission_1 :
         vertex_ids_mission_1_) {
      pose_graph::EdgeId edge_id;
      common::generateId(&edge_id);
      const aslam::Transformation T_G_I_source =
          map_.getVertex_T_G_I(source_vertex_id_mission_2);
      const aslam::Transformation T_G_I_dest =
          map_.getVertex_T_G_I(destination_vertex_id_mission_1);
      const aslam::Transformation T_A_B = T_G_I_source.inverse() * T_G_I_dest;

      LoopClosureEdge::UniquePtr loop_closure_edge =
          aligned_unique<LoopClosureEdge>(
              edge_id, source_vertex_id_mission_2,
              destination_vertex_id_mission_1, kSwitchVariable,
              kSwitchVariableVariance, T_A_B, kT_A_B_covariance);
      map_.addEdge(std::move(loop_closure_edge));
    }
  }
}

// Orphaned since not connected to the graph already attached to mission_id.
void MapConsistencyCheckTest::addOrphanedTrajectory() {
  pose_graph::VertexId vertex_id_0;
  pose_graph::VertexId vertex_id_1;
  generateId(&vertex_id_0);
  generateId(&vertex_id_1);
  vi_map::Vertex* vertex_0(new vi_map::Vertex(cameras_));
  vi_map::Vertex* vertex_1(new vi_map::Vertex(cameras_));
  vertex_0->setId(vertex_id_0);
  vertex_1->setId(vertex_id_1);
  vertex_0->setMissionId(mission_id1_);
  vertex_1->setMissionId(mission_id1_);
  map_.addVertex(vi_map::Vertex::UniquePtr(vertex_0));
  map_.addVertex(vi_map::Vertex::UniquePtr(vertex_1));

  pose_graph::EdgeId edge_id;
  generateId(&edge_id);
  Eigen::Matrix<int64_t, 1, Eigen::Dynamic> imu_timestamps;
  Eigen::Matrix<double, 6, Eigen::Dynamic> imu_data;

  vi_map::Edge* edge(
      new vi_map::ViwlsEdge(
          edge_id, vertex_id_0, vertex_id_1, imu_timestamps, imu_data));
  map_.addEdge(vi_map::Edge::UniquePtr(edge));
}

// Orphaned since not connected to the graph already attached to mission_id.
void MapConsistencyCheckTest::addOrphanedVertex() {
  pose_graph::VertexId vertex_id;
  generateId(&vertex_id);
  vi_map::Vertex* vertex(new vi_map::Vertex(cameras_));
  vertex->setId(vertex_id);
  vertex->setMissionId(mission_id1_);
  map_.addVertex(vi_map::Vertex::UniquePtr(vertex));
}

void MapConsistencyCheckTest::addInvalidEdge() {
  pose_graph::EdgeId edge_id;
  generateId(&edge_id);
  Eigen::Matrix<int64_t, 1, Eigen::Dynamic> imu_timestamps;
  Eigen::Matrix<double, 6, Eigen::Dynamic> imu_data;
  vi_map::Edge* edge(
      new vi_map::ViwlsEdge(
          edge_id, vertex_ids_mission_1_[2], vertex_ids_mission_1_[0],
          imu_timestamps, imu_data));
  map_.addEdge(vi_map::Edge::UniquePtr(edge));
}

void MapConsistencyCheckTest::addSecondMission() {}

void MapConsistencyCheckTest::populatePosegraph(
    const MissionId& mission_id,
    std::vector<pose_graph::VertexId>* vertex_ids) {
  CHECK_NOTNULL(vertex_ids)->clear();
  CHECK(mission_id.isValid());
  pose_graph::VertexId vertex_id;
  vi_map::Vertex* vertex(new vi_map::Vertex(cameras_));
  addVisualFrameToVertex(cameras_, vertex);
  vertex_ids->resize(kNumOfVertices);

  generateId(&vertex_id);
  // Creates mission and sets root vertex id.
  createMission(vertex_id, mission_id, cameras_);

  vertex->setId(vertex_id);
  vertex->setMissionId(mission_id);
  VLOG(3) << "Add vertex " << vertex_id.hexString();
  (*vertex_ids)[0] = vertex_id;
  map_.addVertex(vi_map::Vertex::UniquePtr(vertex));

  for (unsigned int i = 1; i < kNumOfVertices; ++i) {
    vi_map::Vertex* vertex(new vi_map::Vertex(cameras_));
    addVisualFrameToVertex(cameras_, vertex);
    generateId(&vertex_id);
    vertex->setId(vertex_id);
    vertex->setMissionId(mission_id);
    VLOG(3) << "Add vertex " << vertex_id.hexString();
    map_.addVertex(vi_map::Vertex::UniquePtr(vertex));
    (*vertex_ids)[i] = vertex_id;

    pose_graph::EdgeId edge_id;
    generateId(&edge_id);
    Eigen::Matrix<int64_t, 1, Eigen::Dynamic> imu_timestamps;
    Eigen::Matrix<double, 6, Eigen::Dynamic> imu_data;
    vi_map::Edge* edge(
        new vi_map::ViwlsEdge(
            edge_id, (*vertex_ids)[i - 1], (*vertex_ids)[i], imu_timestamps,
            imu_data));
    map_.addEdge(vi_map::Edge::UniquePtr(edge));

    addCommonLandmarkIds(5, (*vertex_ids)[i], (*vertex_ids)[i - 1]);
  }
}

void MapConsistencyCheckTest::addVisualFrameToVertex(
    const aslam::NCamera::Ptr& n_camera_ptr, vi_map::Vertex* vertex_ptr) {
  CHECK(vertex_ptr != nullptr);

  const unsigned int num_cameras = n_camera_ptr->numCameras();

  for (unsigned int j = 0; j < num_cameras; ++j) {
    aslam::VisualFrame::Ptr frame(new aslam::VisualFrame);
    Eigen::Matrix2Xd img_points_distorted;
    Eigen::VectorXd uncertainties;
    aslam::VisualFrame::DescriptorsT descriptors;
    img_points_distorted.resize(Eigen::NoChange, kNumOfKeypointsPerVertex);
    uncertainties.resize(kNumOfKeypointsPerVertex);
    descriptors.resize(kDescriptorBytes, kNumOfKeypointsPerVertex);
    aslam::FrameId frame_id;
    common::generateId(&frame_id);
    frame->setId(frame_id);
    frame->setKeypointMeasurements(img_points_distorted);
    frame->setKeypointMeasurementUncertainties(uncertainties);
    frame->setDescriptors(descriptors);
    frame->setCameraGeometry(n_camera_ptr->getCameraShared(j));
    vertex_ptr->getVisualNFrame().setFrame(j, frame);

    LOG(INFO) << "Add global landmark ids to frame: " << j;
    for (unsigned int i = 0; i < kNumOfKeypointsPerVertex; ++i) {
      vertex_ptr->addObservedLandmarkId(j, vi_map::LandmarkId());
    }
  }
}

void MapConsistencyCheckTest::createMission(
    const pose_graph::VertexId& root_vertex_id,
    const vi_map::MissionId& mission_id,
    const aslam::NCamera::Ptr& n_camera_ptr) {
  vi_map::MissionBaseFrame baseframe;
  vi_map::MissionBaseFrameId baseframe_id;
  common::generateId(&baseframe_id);
  baseframe.setId(baseframe_id);

  vi_map::VIMission* mission_ptr(new vi_map::VIMission);

  mission_ptr->setId(mission_id);
  mission_ptr->setBaseFrameId(baseframe_id);

  mission_ptr->setRootVertexId(root_vertex_id);
  map_.missions.emplace(
      mission_ptr->id(), vi_map::VIMission::UniquePtr(mission_ptr));
  map_.mission_base_frames.emplace(baseframe.id(), baseframe);
  map_.sensor_manager_.addNCamera(n_camera_ptr, mission_id);
}

void MapConsistencyCheckTest::addLandmarkAndVertexReference(
    const vi_map::LandmarkId& landmark_id,
    const pose_graph::VertexId& storing_vertex_id) {
  map_.addLandmarkIndexReference(landmark_id, storing_vertex_id);
}

void MapConsistencyCheckTest::addCommonLandmarkIds(
    unsigned int count, const pose_graph::VertexId& vertex0_id,
    const pose_graph::VertexId& vertex1_id) {
  vi_map::Vertex& vertex0 = map_.getVertex(vertex0_id);
  vi_map::Vertex& vertex1 = map_.getVertex(vertex1_id);

  vi_map::LandmarkStore& landmark_store = vertex0.getLandmarks();

  for (unsigned int i = 0; i < count; ++i) {
    vi_map::LandmarkId landmark_id;
    generateId(&landmark_id);

    // Set every 10th GlobalLandmarkId in a vertex to a valid ID.
    const unsigned int keypoint_index0 =
        10 * vertex0.numValidObservedLandmarkIds(kVisualFrameIndex);
    const unsigned int keypoint_index1 =
        10 * vertex1.numValidObservedLandmarkIds(kVisualFrameIndex);

    vi_map::Landmark landmark;
    landmark.setId(landmark_id);
    landmark.addObservation(vertex0_id, kVisualFrameIndex, keypoint_index0);
    landmark.addObservation(vertex1_id, kVisualFrameIndex, keypoint_index1);
    landmark_store.addLandmark(landmark);

    CHECK_LT(
        keypoint_index0, vertex1.observedLandmarkIdsSize(kVisualFrameIndex));
    CHECK_LT(
        keypoint_index1, vertex0.observedLandmarkIdsSize(kVisualFrameIndex));
    vertex0.setObservedLandmarkId(
        kVisualFrameIndex, keypoint_index0, landmark_id);
    vertex1.setObservedLandmarkId(
        kVisualFrameIndex, keypoint_index1, landmark_id);

    map_.addLandmarkIndexReference(landmark_id, vertex0_id);

    VLOG(3) << "Add landmark " << landmark_id.hexString() << " to "
            << vertex0_id.hexString() << " and " << vertex1_id.hexString()
            << " pos " << keypoint_index0 << " and " << keypoint_index1;
  }
}

void MapConsistencyCheckTest::addMissionWithVisualNFrame() {
  static constexpr unsigned int kNumCamerasInMultiframeMission = 5;
  aslam::NCamera::Ptr n_cameras =
      aslam::NCamera::createTestNCamera(kNumCamerasInMultiframeMission);

  pose_graph::VertexId vertex_id;
  vi_map::Vertex* vertex(new vi_map::Vertex(n_cameras));

  common::generateId(&mission_id1_);
  common::generateId(&vertex_id);
  // Creates mission and sets root vertex id.
  createMission(vertex_id, mission_id1_, n_cameras);

  vertex->setId(vertex_id);
  vertex->setMissionId(mission_id1_);
  addVisualFrameToVertex(n_cameras, vertex);
  map_.addVertex(vi_map::Vertex::UniquePtr(vertex));
}

void MapConsistencyCheckTest::addMissionUsingAnExistingNCamera(
    const aslam::NCamera::Ptr& n_cameras) {
  CHECK(n_cameras);
  CHECK(map_.getSensorManager().hasSensor(n_cameras->getId()));

  pose_graph::VertexId vertex_id;
  vi_map::Vertex* vertex(new vi_map::Vertex(n_cameras));

  common::generateId(&mission_id1_);
  common::generateId(&vertex_id);
  // Creates mission and sets root vertex id.
  createMission(vertex_id, mission_id1_, n_cameras);

  vertex->setId(vertex_id);
  vertex->setMissionId(mission_id1_);
  addVisualFrameToVertex(n_cameras, vertex);
  map_.addVertex(vi_map::Vertex::UniquePtr(vertex));
}

TEST_F(MapConsistencyCheckTest, mapConsistencySingleVisualFrame) {
  EXPECT_TRUE(vi_map::checkMapConsistency(map_));
}

TEST_F(MapConsistencyCheckTest, mapConsistencyMultiVisualFrame) {
  addMissionWithVisualNFrame();
  EXPECT_TRUE(vi_map::checkMapConsistency(map_));
}

TEST_F(MapConsistencyCheckTest, mapInconsistentMissingBackLink) {
  vi_map::Vertex& vertex0 = map_.getVertex(vertex_ids_mission_1_[2]);
  vi_map::LandmarkStore& landmark_store = vertex0.getLandmarks();
  ASSERT_GT(landmark_store.size(), 0);

  vi_map::LandmarkId landmark_id;
  generateId(&landmark_id);

  vi_map::Landmark landmark;
  landmark.setId(landmark_id);
  landmark_store.addLandmark(landmark);
  addLandmarkAndVertexReference(landmark_id, vertex_ids_mission_1_[2]);

  const unsigned int keypoint_index =
      vertex0.numValidObservedLandmarkIds(kVisualFrameIndex);
  vertex0.setObservedLandmarkId(kVisualFrameIndex, keypoint_index, landmark_id);

  EXPECT_FALSE(vi_map::checkMapConsistency(map_));
}

TEST_F(MapConsistencyCheckTest, mapInconsistentPosegraphInvalidEdge) {
  addInvalidEdge();
  EXPECT_FALSE(vi_map::checkMapConsistency(map_));
}

TEST_F(MapConsistencyCheckTest, mapInconsistentPosegraphOrphanedTrajectory) {
  addOrphanedTrajectory();
  EXPECT_FALSE(vi_map::checkMapConsistency(map_));
}

TEST_F(MapConsistencyCheckTest, mapInconsistentPosegraphOrphanedVertex) {
  addOrphanedVertex();
  EXPECT_FALSE(vi_map::checkMapConsistency(map_));
}

TEST_F(MapConsistencyCheckTest, mapDuplicatedMissionNCameraReference) {
  addMissionUsingAnExistingNCamera(cameras_);
  EXPECT_TRUE(vi_map::checkMapConsistency(map_));
}

TEST_F(MapConsistencyCheckTest, MapConsistencyLoopClosureEdges) {
  addMissionWithVisualNFrame();
  EXPECT_TRUE(vi_map::checkMapConsistency(map_));
  addLoopClosureEdges();
  EXPECT_TRUE(vi_map::checkMapConsistency(map_));
}

}  // namespace vi_map

MAPLAB_UNITTEST_ENTRYPOINT
