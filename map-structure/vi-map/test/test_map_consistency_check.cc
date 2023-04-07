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
  virtual void SetUp() {}

  void createMissionWithExistingNCamera(
      const vi_map::MissionId& mission_id, const aslam::SensorId& ncamera_id,
      std::vector<pose_graph::VertexId>* vertex_ids,
      const unsigned int num_vertices = kNumOfVertices);
  void createMissionWithNewNCamera(
      const vi_map::MissionId& mission_id,
      std::vector<pose_graph::VertexId>* vertex_ids,
      const unsigned int num_vertices = kNumOfVertices);
  void populateVisualNFrameOfVertex(vi_map::Vertex* vertex_ptr);

  void addVisualFrameToVertex(
      const aslam::NCamera::Ptr& n_camera_ptr, vi_map::Vertex* vertex_ptr);
  void addCommonLandmarkIds(
      unsigned int count, const pose_graph::VertexId& vertex0_id,
      const pose_graph::VertexId& vertex1_id);
  void addInvalidEdge();
  void addOrphanedTrajectory();
  void addOrphanedVertex();
  void addLandmarkAndVertexReference(
      const vi_map::LandmarkId& global_landmark_id,
      const pose_graph::VertexId& storing_vertex_id);
  void addLoopClosureEdges();

  std::vector<pose_graph::VertexId> vertex_ids_mission_1_;
  std::vector<pose_graph::VertexId> vertex_ids_mission_2_;
  VIMap map_;
  vi_map::MissionId mission_id1_;
  vi_map::MissionId mission_id2_;

  static constexpr unsigned int kNumOfVertices = 4;
  static constexpr unsigned int kNumCameras = 1;
  static constexpr unsigned int kVisualFrameIndex = 0;
  static constexpr unsigned int kNumOfKeypointsPerVertex = 500;
  static constexpr int kDescriptorBytes = 48;
};

void MapConsistencyCheckTest::createMissionWithExistingNCamera(
    const vi_map::MissionId& mission_id, const aslam::SensorId& ncamera_id,
    std::vector<pose_graph::VertexId>* vertex_ids,
    const unsigned int num_vertices) {
  CHECK(map_.getSensorManager().hasSensor(ncamera_id));
  CHECK_EQ(
      map_.getSensorManager().getSensorType(ncamera_id),
      vi_map::SensorType::kNCamera);
  CHECK(mission_id.isValid());
  CHECK_NOTNULL(vertex_ids)->clear();

  // Retrieve the sensor from the sensor manager.
  const aslam::NCamera::Ptr& ncamera_ptr =
      map_.sensor_manager_.getSensorPtr<aslam::NCamera>(ncamera_id);

  vi_map::MissionBaseFrame baseframe;
  vi_map::MissionBaseFrameId baseframe_id;
  aslam::generateId(&baseframe_id);
  baseframe.setId(baseframe_id);

  vi_map::VIMission* mission_ptr(new vi_map::VIMission);

  mission_ptr->setId(mission_id);
  mission_ptr->setBaseFrameId(baseframe_id);

  map_.missions.emplace(
      mission_ptr->id(), vi_map::VIMission::UniquePtr(mission_ptr));
  map_.mission_base_frames.emplace(baseframe.id(), baseframe);

  // Associate mission with ncamera.
  map_.associateMissionNCamera(ncamera_ptr->getId(), mission_id);

  if (num_vertices == 0u) {
    return;
  }

  // Add vertices_
  vertex_ids->resize(num_vertices);

  // Create and add root vertex.
  vi_map::Vertex::UniquePtr root_vertex =
      aligned_unique<vi_map::Vertex>(ncamera_ptr);
  populateVisualNFrameOfVertex(root_vertex.get());
  const pose_graph::VertexId& root_vertex_id = root_vertex->id();
  CHECK(root_vertex_id.isValid());
  root_vertex->setMissionId(mission_id);
  map_.addVertex(std::move(root_vertex));
  mission_ptr->setRootVertexId(root_vertex_id);

  // Save vertex id for external logic, not needed for map itself.
  (*vertex_ids)[0] = root_vertex_id;
  VLOG(3) << "Add root vertex " << root_vertex_id.hexString();

  for (unsigned int i = 1; i < num_vertices; ++i) {
    vi_map::Vertex::UniquePtr intermediate_vertex =
        aligned_unique<vi_map::Vertex>(ncamera_ptr);
    populateVisualNFrameOfVertex(intermediate_vertex.get());
    const pose_graph::VertexId& intermediate_vertex_id =
        intermediate_vertex->id();
    CHECK(intermediate_vertex_id.isValid());
    intermediate_vertex->setMissionId(mission_id);
    map_.addVertex(std::move(intermediate_vertex));

    (*vertex_ids)[i] = intermediate_vertex_id;
    VLOG(3) << "Add vertex " << intermediate_vertex_id.hexString();

    pose_graph::EdgeId edge_id;
    generateId(&edge_id);
    Eigen::Matrix<int64_t, 1, Eigen::Dynamic> imu_timestamps;
    Eigen::Matrix<double, 6, Eigen::Dynamic> imu_data;
    vi_map::Edge* edge(new vi_map::ViwlsEdge(
        edge_id, (*vertex_ids)[i - 1], (*vertex_ids)[i], imu_timestamps,
        imu_data));
    map_.addEdge(vi_map::Edge::UniquePtr(edge));

    addCommonLandmarkIds(5, (*vertex_ids)[i], (*vertex_ids)[i - 1]);
  }
}

void MapConsistencyCheckTest::createMissionWithNewNCamera(
    const vi_map::MissionId& mission_id,
    std::vector<pose_graph::VertexId>* vertex_ids,
    const unsigned int num_vertices) {
  CHECK(mission_id.isValid());
  CHECK_NOTNULL(vertex_ids)->clear();

  // Add an NCamera.
  static constexpr unsigned int kNumCamerasInMultiframeMission = 5;
  aslam::NCamera::UniquePtr ncamera =
      aslam::createUniqueTestNCamera(kNumCamerasInMultiframeMission);
  const aslam::SensorId& ncamera_id = ncamera->getId();
  map_.sensor_manager_.addSensorAsBase<aslam::NCamera>(std::move(ncamera));
  CHECK(!ncamera);

  createMissionWithExistingNCamera(
      mission_id, ncamera_id, vertex_ids, num_vertices);
}

void MapConsistencyCheckTest::populateVisualNFrameOfVertex(
    vi_map::Vertex* vertex_ptr) {
  CHECK_NOTNULL(vertex_ptr);
  aslam::VisualNFrame& nframe = vertex_ptr->getVisualNFrame();
  const unsigned int num_frames = nframe.getNumFrames();

  CHECK_EQ(nframe.getNumFrames(), num_frames);
  for (unsigned int frame_idx = 0u; frame_idx < num_frames; ++frame_idx) {
    aslam::VisualFrame::Ptr frame = nframe.getFrameShared(frame_idx);

    Eigen::Matrix2Xd img_points_distorted;
    Eigen::VectorXd uncertainties;
    aslam::VisualFrame::DescriptorsT descriptors;
    img_points_distorted.resize(Eigen::NoChange, kNumOfKeypointsPerVertex);
    uncertainties.resize(kNumOfKeypointsPerVertex);
    descriptors.resize(kDescriptorBytes, kNumOfKeypointsPerVertex);

    frame->setKeypointMeasurements(img_points_distorted);
    frame->setKeypointMeasurementUncertainties(uncertainties);
    frame->setDescriptors(descriptors);

    LOG(INFO) << "Add global landmark ids to frame: " << frame_idx;
    for (unsigned int keypoint_idx = 0; keypoint_idx < kNumOfKeypointsPerVertex;
         ++keypoint_idx) {
      vertex_ptr->addObservedLandmarkId(frame_idx, vi_map::LandmarkId());
    }
  }
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

void MapConsistencyCheckTest::addLoopClosureEdges() {
  aslam::generateId(&mission_id2_);
  createMissionWithNewNCamera(mission_id2_, &vertex_ids_mission_2_);

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
      aslam::generateId(&edge_id);
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
      aslam::generateId(&edge_id);
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
      aslam::generateId(&edge_id);
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
      aslam::generateId(&edge_id);
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
  aslam::NCamera::Ptr ncamera = map_.getMissionNCameraPtr(mission_id1_);

  pose_graph::VertexId vertex_id_0;
  pose_graph::VertexId vertex_id_1;
  generateId(&vertex_id_0);
  generateId(&vertex_id_1);
  vi_map::Vertex* vertex_0(new vi_map::Vertex(ncamera));
  vi_map::Vertex* vertex_1(new vi_map::Vertex(ncamera));
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

  vi_map::Edge* edge(new vi_map::ViwlsEdge(
      edge_id, vertex_id_0, vertex_id_1, imu_timestamps, imu_data));
  map_.addEdge(vi_map::Edge::UniquePtr(edge));
}

// Orphaned since not connected to the graph already attached to mission_id.
void MapConsistencyCheckTest::addOrphanedVertex() {
  aslam::NCamera::Ptr ncamera = map_.getMissionNCameraPtr(mission_id1_);

  pose_graph::VertexId vertex_id;
  generateId(&vertex_id);
  vi_map::Vertex* vertex(new vi_map::Vertex(ncamera));
  vertex->setId(vertex_id);
  vertex->setMissionId(mission_id1_);
  map_.addVertex(vi_map::Vertex::UniquePtr(vertex));
}

void MapConsistencyCheckTest::addInvalidEdge() {
  pose_graph::EdgeId edge_id;
  generateId(&edge_id);
  Eigen::Matrix<int64_t, 1, Eigen::Dynamic> imu_timestamps;
  Eigen::Matrix<double, 6, Eigen::Dynamic> imu_data;
  vi_map::Edge* edge(new vi_map::ViwlsEdge(
      edge_id, vertex_ids_mission_1_[2], vertex_ids_mission_1_[0],
      imu_timestamps, imu_data));
  map_.addEdge(vi_map::Edge::UniquePtr(edge));
}

void MapConsistencyCheckTest::addLandmarkAndVertexReference(
    const vi_map::LandmarkId& landmark_id,
    const pose_graph::VertexId& storing_vertex_id) {
  map_.addLandmarkIndexReference(landmark_id, storing_vertex_id);
}

TEST_F(MapConsistencyCheckTest, mapConsistencyNoVertex) {
  aslam::generateId(&mission_id1_);
  createMissionWithNewNCamera(
      mission_id1_, &vertex_ids_mission_1_, 0u /* num vertices */);
  EXPECT_FALSE(vi_map::checkMapConsistency(map_));
}

TEST_F(MapConsistencyCheckTest, mapConsistencySingleVertex) {
  aslam::generateId(&mission_id1_);
  createMissionWithNewNCamera(
      mission_id1_, &vertex_ids_mission_1_, 1u /* num vertices */);
  EXPECT_TRUE(vi_map::checkMapConsistency(map_));
}

TEST_F(MapConsistencyCheckTest, mapConsistencyMultipleVertices) {
  aslam::generateId(&mission_id1_);
  createMissionWithNewNCamera(
      mission_id1_, &vertex_ids_mission_1_, 5u /* num vertices */);
  EXPECT_TRUE(vi_map::checkMapConsistency(map_));
}

TEST_F(MapConsistencyCheckTest, mapInconsistentMissingBackLink) {
  aslam::generateId(&mission_id1_);
  createMissionWithNewNCamera(mission_id1_, &vertex_ids_mission_1_);

  vi_map::Vertex& vertex0 = map_.getVertex(vertex_ids_mission_1_[2]);
  vi_map::LandmarkStore& landmark_store = vertex0.getLandmarks();
  ASSERT_GT(landmark_store.size(), 0u);

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
  aslam::generateId(&mission_id1_);
  createMissionWithNewNCamera(mission_id1_, &vertex_ids_mission_1_);

  addInvalidEdge();
  EXPECT_FALSE(vi_map::checkMapConsistency(map_));
}

TEST_F(MapConsistencyCheckTest, mapInconsistentPosegraphOrphanedTrajectory) {
  aslam::generateId(&mission_id1_);
  createMissionWithNewNCamera(mission_id1_, &vertex_ids_mission_1_);

  addOrphanedTrajectory();
  EXPECT_FALSE(vi_map::checkMapConsistency(map_));
}

TEST_F(MapConsistencyCheckTest, mapInconsistentPosegraphOrphanedVertex) {
  aslam::generateId(&mission_id1_);
  createMissionWithNewNCamera(mission_id1_, &vertex_ids_mission_1_);

  addOrphanedVertex();
  EXPECT_FALSE(vi_map::checkMapConsistency(map_));
}

TEST_F(MapConsistencyCheckTest, mapMultiMissionDifferentNCamera) {
  aslam::generateId(&mission_id1_);
  createMissionWithNewNCamera(mission_id1_, &vertex_ids_mission_1_);

  aslam::generateId(&mission_id2_);
  createMissionWithNewNCamera(mission_id2_, &vertex_ids_mission_2_);

  EXPECT_TRUE(vi_map::checkMapConsistency(map_));
}

TEST_F(MapConsistencyCheckTest, mapMultiMissionSameNCamera) {
  aslam::generateId(&mission_id1_);
  createMissionWithNewNCamera(mission_id1_, &vertex_ids_mission_1_);

  const aslam::NCamera::Ptr ncamera_ptr =
      map_.getMissionNCameraPtr(mission_id1_);

  aslam::generateId(&mission_id2_);
  createMissionWithExistingNCamera(
      mission_id2_, ncamera_ptr->getId(), &vertex_ids_mission_2_);

  EXPECT_TRUE(vi_map::checkMapConsistency(map_));
}

TEST_F(MapConsistencyCheckTest, mapMissionWithInconsistentPointersToNCamera) {
  aslam::generateId(&mission_id1_);
  createMissionWithNewNCamera(mission_id1_, &vertex_ids_mission_1_);

  const aslam::NCamera::Ptr ncamera_copy_ptr =
      aligned_shared<aslam::NCamera>(map_.getMissionNCamera(mission_id1_));

  for (const pose_graph::VertexId& vertex_id : vertex_ids_mission_1_) {
    map_.getVertex(vertex_id).getVisualNFrameShared()->setNCameras(
        ncamera_copy_ptr);
  }

  EXPECT_FALSE(vi_map::checkMapConsistency(map_));
}

TEST_F(MapConsistencyCheckTest, MapConsistencyLoopClosureEdges) {
  aslam::generateId(&mission_id1_);
  createMissionWithNewNCamera(mission_id1_, &vertex_ids_mission_1_);
  EXPECT_TRUE(vi_map::checkMapConsistency(map_));
  addLoopClosureEdges();
  EXPECT_TRUE(vi_map::checkMapConsistency(map_));
}

}  // namespace vi_map

MAPLAB_UNITTEST_ENTRYPOINT
