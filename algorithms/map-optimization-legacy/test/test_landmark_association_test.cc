#include <memory>
#include <unordered_map>
#include <unordered_set>

#include <glog/logging.h>
#include <maplab-common/test/testing-entrypoint.h>
#include <maplab-common/test/testing-predicates.h>
#include <maplab-common/unique-id.h>
#include <vi-map/check-map-consistency.h>
#include <vi-map/pose-graph.h>
#include <vi-map/vi-map.h>

namespace map_optimization_legacy {

class ViwlsGraph : public ::testing::Test {
 protected:
  ViwlsGraph() {}

  virtual void SetUp() {
    cameras_ = aslam::NCamera::createTestNCamera(1);

    addVertices();
    addStoreLandmarksAndKeypoints();
    addUnassociatedObservations();
  }

  void addVertices();
  void addStoreLandmarksAndKeypoints();
  void addUnassociatedObservations();

  vi_map::VIMap vi_map_;
  pose_graph::VertexIdList vertex_ids_;

  aslam::NCamera::Ptr cameras_;

  static constexpr unsigned int kNumOfVertices = 5;
  static constexpr unsigned int kNumOfStoreLandmarks = 5;
  static constexpr unsigned int kNumOfKeypointsPerVertex =
      kNumOfStoreLandmarks + 1;
  static constexpr unsigned int kDescriptorBytes = 48;
  static constexpr unsigned int kNumCameras = 1;
  static constexpr unsigned int kVisualFrameIndex = 0;
};

void ViwlsGraph::addVertices() {
  vertex_ids_.clear();
  vi_map::MissionId mission_id;
  generateId(&mission_id);

  vi_map::Vertex::UniquePtr vertex(new vi_map::Vertex(cameras_));
  pose_graph::VertexId vertex_id;
  generateId(&vertex_id);

  vertex->setId(vertex_id);
  vertex->setMissionId(mission_id);
  vertex_ids_.push_back(vertex_id);

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
  frame->setCameraGeometry(cameras_->getCameraShared(kVisualFrameIndex));
  vertex->getVisualNFrame().setFrame(kVisualFrameIndex, frame);
  CHECK_EQ(kNumCameras, vertex->numFrames());

  vi_map::MissionBaseFrameId baseframe_id;
  generateId(&baseframe_id);
  vi_map::MissionBaseFrame mission_baseframe;
  mission_baseframe.setId(baseframe_id);
  vi_map_.mission_base_frames.emplace(baseframe_id, mission_baseframe);

  vi_map::VIMission::UniquePtr mission(new vi_map::VIMission);
  mission->setId(mission_id);
  mission->setRootVertexId(vertex_ids_.front());
  mission->setBaseFrameId(baseframe_id);
  vi_map_.missions.emplace(mission_id, std::move(mission));

  vi_map_.sensor_manager_.addNCamera(cameras_, mission_id);

  vi_map_.addVertex(std::move(vertex));

  for (unsigned int i = 1; i < kNumOfVertices; ++i) {
    pose_graph::VertexId vertex_id;
    vi_map::Vertex::UniquePtr vertex(new vi_map::Vertex(cameras_));
    generateId(&vertex_id);
    vertex->setId(vertex_id);
    vertex->setMissionId(mission_id);
    aslam::VisualFrame::Ptr frame(new aslam::VisualFrame);
    aslam::FrameId frame_id;
    common::generateId(&frame_id);
    frame->setId(frame_id);
    frame->setKeypointMeasurements(img_points_distorted);
    frame->setKeypointMeasurementUncertainties(uncertainties);
    frame->setDescriptors(descriptors);
    frame->setCameraGeometry(cameras_->getCameraShared(kVisualFrameIndex));
    vertex->getVisualNFrame().setFrame(kVisualFrameIndex, frame);
    CHECK_EQ(kNumCameras, vertex->numFrames());

    vi_map_.addVertex(std::move(vertex));
    vertex_ids_.push_back(vertex_id);

    pose_graph::EdgeId edge_id;
    generateId(&edge_id);

    Eigen::Matrix<int64_t, 1, Eigen::Dynamic> imu_timestamps;
    Eigen::Matrix<double, 6, Eigen::Dynamic> imu_data;

    vi_map::ViwlsEdge::UniquePtr edge(
        new vi_map::ViwlsEdge(
            edge_id, vertex_ids_[i - 1], vertex_ids_[i], imu_timestamps,
            imu_data));
    vi_map_.addEdge(std::move(edge));
  }
}

void ViwlsGraph::addStoreLandmarksAndKeypoints() {
  CHECK(!vertex_ids_.empty());
  pose_graph::VertexId vertex_id_storing_landmark = vertex_ids_[0];

  for (unsigned int i = 0; i < kNumOfStoreLandmarks; ++i) {
    vi_map::LandmarkId landmark_id;
    common::generateId(&landmark_id);

    vi_map::Landmark store_landmark;
    store_landmark.setId(landmark_id);
    CHECK(!vertex_ids_.empty());
    vi_map_.getVertex(vertex_id_storing_landmark)
        .getLandmarks()
        .addLandmark(store_landmark);

    for (unsigned int k = 0; k < kNumOfVertices; ++k) {
      vi_map_.getVertex(vertex_ids_[k])
          .addObservedLandmarkId(kVisualFrameIndex, landmark_id);
      vi_map_.getVertex(vertex_id_storing_landmark)
          .getLandmarks()
          .getLandmark(landmark_id)
          .addObservation(vertex_ids_[k], kVisualFrameIndex, i);
    }

    vi_map_.landmark_index.addLandmarkAndVertexReference(
        landmark_id, vertex_id_storing_landmark);
  }
}

void ViwlsGraph::addUnassociatedObservations() {
  for (const pose_graph::VertexId& vertex_id : vertex_ids_) {
    vi_map::LandmarkId landmark_id;
    landmark_id.setInvalid();
    vi_map_.getVertex(vertex_id).addObservedLandmarkId(
        kVisualFrameIndex, landmark_id);
  }
}

TEST_F(ViwlsGraph, AddNewLandmarkTest) {
  CHECK(!vertex_ids_.empty());

  vi_map::LandmarkId landmark_id;
  common::generateId(&landmark_id);
  vi_map::Landmark new_landmark;
  new_landmark.setId(landmark_id);

  vi_map_.addNewLandmark(
      new_landmark, vertex_ids_[0], kVisualFrameIndex,
      kNumOfKeypointsPerVertex - 1);
  EXPECT_TRUE(checkMapConsistency(vi_map_));
}

TEST_F(ViwlsGraph, AssociateKeypointWithExistingLandmarkTest) {
  CHECK(!vertex_ids_.empty());

  vi_map::LandmarkId landmark_id;
  common::generateId(&landmark_id);
  vi_map::Landmark new_landmark;
  new_landmark.setId(landmark_id);

  vi_map_.addNewLandmark(
      new_landmark, vertex_ids_[0], kVisualFrameIndex,
      kNumOfKeypointsPerVertex - 1);
  for (unsigned int i = 1; i < vertex_ids_.size(); ++i) {
    vi_map_.associateKeypointWithExistingLandmark(
        vertex_ids_[i], kVisualFrameIndex, kNumOfKeypointsPerVertex - 1,
        landmark_id);
  }
  EXPECT_TRUE(checkMapConsistency(vi_map_));
}

TEST_F(ViwlsGraph, AssociateDuplicateLandmarkTest) {
  CHECK(!vertex_ids_.empty());

  vi_map::LandmarkId landmark_id;
  common::generateId(&landmark_id);
  vi_map::Landmark new_landmark;
  new_landmark.setId(landmark_id);

  vi_map_.addNewLandmark(
      new_landmark, vertex_ids_[0], kVisualFrameIndex,
      kNumOfKeypointsPerVertex - 1);
  EXPECT_DEATH(
      vi_map_.addNewLandmark(
          new_landmark, vertex_ids_[0], kVisualFrameIndex,
          kNumOfKeypointsPerVertex - 1),
      "Check failed");
}

TEST_F(ViwlsGraph, AssociateLandmarkToAlreadyAssociatedKeypointTest) {
  CHECK(!vertex_ids_.empty());

  vi_map::LandmarkId landmark_id;
  common::generateId(&landmark_id);
  vi_map::Landmark new_landmark;
  new_landmark.setId(landmark_id);

  EXPECT_DEATH(
      vi_map_.addNewLandmark(
          new_landmark, vertex_ids_[0], kVisualFrameIndex,
          kNumOfKeypointsPerVertex - 2),
      "Check failed");
}

TEST_F(ViwlsGraph, AssociateToNonexistentVertexTest) {
  CHECK(!vertex_ids_.empty());

  vi_map::LandmarkId landmark_id;
  common::generateId(&landmark_id);
  vi_map::Landmark new_landmark;
  new_landmark.setId(landmark_id);

  pose_graph::VertexId dummy_vertex_id;
  generateId(&dummy_vertex_id);
  EXPECT_DEATH(
      vi_map_.addNewLandmark(
          new_landmark, dummy_vertex_id, kVisualFrameIndex,
          kNumOfKeypointsPerVertex - 2),
      "Check failed");
}

TEST_F(ViwlsGraph, AssociateToWrongKeypointIndexTest) {
  CHECK(!vertex_ids_.empty());

  vi_map::LandmarkId landmark_id;
  common::generateId(&landmark_id);
  vi_map::Landmark new_landmark;
  new_landmark.setId(landmark_id);

  pose_graph::VertexId dummy_vertex_id;
  generateId(&dummy_vertex_id);
  EXPECT_DEATH(
      vi_map_.addNewLandmark(
          new_landmark, vertex_ids_[0], kVisualFrameIndex,
          kNumOfKeypointsPerVertex + 5),
      "Check failed");
}

}  // namespace map_optimization_legacy

MAPLAB_UNITTEST_ENTRYPOINT
