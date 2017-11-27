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

    CHECK(!landmark_ids_.empty());

    // Fix seed for Eigen random functions.
    srand(42);
  }

  void addVisualFrameToVertex(vi_map::Vertex* vertex_ptr);
  void addVertices();
  void addStoreLandmarksAndKeypoints();

  vi_map::VIMap vi_map_;
  pose_graph::VertexIdList vertex_ids_;
  vi_map::LandmarkIdList landmark_ids_;

  aslam::NCamera::Ptr cameras_;

  static constexpr unsigned int kNumOfVertices = 2;
  static constexpr unsigned int kNumOfLandmarksPerStoreLandmark = 2;
  static constexpr unsigned int kNumOfStoreLandmarks = 2;
  static constexpr unsigned int kNumOfKeypointsPerVertex =
      kNumOfStoreLandmarks * kNumOfLandmarksPerStoreLandmark;
  static constexpr int kDescriptorBytes = 48;
  static constexpr int kNumCameras = 1;
  static constexpr int kVisualFrameIndex = 0;
};

void ViwlsGraph::addVisualFrameToVertex(vi_map::Vertex* vertex_ptr) {
  CHECK_NOTNULL(vertex_ptr);

  aslam::VisualFrame::Ptr frame(new aslam::VisualFrame);
  Eigen::Matrix2Xd img_points_distorted;
  Eigen::VectorXd uncertainties;
  aslam::VisualFrame::DescriptorsT descriptors;
  img_points_distorted.resize(Eigen::NoChange, kNumOfKeypointsPerVertex);
  img_points_distorted.setRandom();
  uncertainties.resize(kNumOfKeypointsPerVertex);
  descriptors.resize(kDescriptorBytes, kNumOfKeypointsPerVertex);
  aslam::FrameId frame_id;
  common::generateId(&frame_id);
  frame->setId(frame_id);
  frame->setKeypointMeasurements(img_points_distorted);
  frame->setKeypointMeasurementUncertainties(uncertainties);
  frame->setDescriptors(descriptors);
  frame->setCameraGeometry(cameras_->getCameraShared(kVisualFrameIndex));
  vertex_ptr->getVisualNFrame().setFrame(kVisualFrameIndex, frame);
}

void ViwlsGraph::addVertices() {
  vertex_ids_.clear();
  vi_map::MissionId mission_id;
  generateId(&mission_id);

  vi_map::Vertex::UniquePtr vertex(new vi_map::Vertex(cameras_));
  pose_graph::VertexId vertex_id;
  generateId(&vertex_id);

  vertex->setId(vertex_id);
  vertex->setMissionId(mission_id);
  VLOG(3) << "Add vertex " << vertex_id.hexString();
  vertex_ids_.push_back(vertex_id);

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

  addVisualFrameToVertex(vertex.get());
  vi_map_.addVertex(std::move(vertex));

  for (unsigned int i = 1; i < kNumOfVertices; ++i) {
    pose_graph::VertexId vertex_id;
    vi_map::Vertex::UniquePtr vertex(new vi_map::Vertex(cameras_));
    generateId(&vertex_id);
    vertex->setId(vertex_id);
    vertex->setMissionId(mission_id);
    addVisualFrameToVertex(vertex.get());
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
  landmark_ids_.clear();

  CHECK(!vertex_ids_.empty());
  pose_graph::VertexId vertex_id_storing_landmark = vertex_ids_[0];

  for (unsigned int i = 0; i < kNumOfStoreLandmarks; ++i) {
    vi_map::LandmarkId landmark_id;
    common::generateId(&landmark_id);
    landmark_ids_.push_back(landmark_id);

    vi_map::Landmark landmark;
    landmark.setId(landmark_id);
    CHECK(!vertex_ids_.empty());
    vi_map_.getVertex(vertex_id_storing_landmark)
        .getLandmarks()
        .addLandmark(landmark);

    vi_map_.landmark_index.addLandmarkAndVertexReference(
        landmark_id, vertex_id_storing_landmark);

    for (unsigned int j = 0; j < kNumOfLandmarksPerStoreLandmark; ++j) {
      for (unsigned int k = 0; k < kNumOfVertices; ++k) {
        vi_map_.getVertex(vertex_ids_[k])
            .addObservedLandmarkId(kVisualFrameIndex, landmark_id);
        vi_map_.getVertex(vertex_id_storing_landmark)
            .getLandmarks()
            .getLandmark(landmark_id)
            .addObservation(
                vertex_ids_[k], kVisualFrameIndex,
                i * kNumOfStoreLandmarks + j);
      }
    }
  }
}

TEST_F(ViwlsGraph, MergeTest) {
  CHECK_GE(landmark_ids_.size(), 2u);
  const vi_map::LandmarkId& store_landmark_id0 = landmark_ids_[0];
  const vi_map::LandmarkId& store_landmark_id1 = landmark_ids_[1];

  EXPECT_TRUE(checkMapConsistency(vi_map_));
  vi_map_.mergeLandmarks(store_landmark_id1, store_landmark_id0);
  EXPECT_TRUE(checkMapConsistency(vi_map_));
}

TEST_F(ViwlsGraph, MergedLandmarkDeletionTest) {
  // General idea: merge landmark 0 and 1 together. All observations
  // for landmark 0 should now point to landmark 1.
  CHECK_GE(landmark_ids_.size(), 2u);
  const vi_map::LandmarkId& store_landmark_id0 = landmark_ids_[0];
  const vi_map::LandmarkId& store_landmark_id1 = landmark_ids_[1];

  const vi_map::Landmark& landmark0 = vi_map_.getLandmark(store_landmark_id0);
  std::vector<vi_map::KeypointIdentifier> landmark0_observations;
  landmark0.forEachObservation(
      [&](const vi_map::KeypointIdentifier& keypoint_id) {
        landmark0_observations.push_back(keypoint_id);
      });

  ASSERT_TRUE(checkMapConsistency(vi_map_));
  vi_map_.mergeLandmarks(store_landmark_id0, store_landmark_id1);
  ASSERT_TRUE(checkMapConsistency(vi_map_));

  EXPECT_FALSE(vi_map_.hasLandmark(store_landmark_id0));
  for (const vi_map::KeypointIdentifier& obs : landmark0_observations) {
    EXPECT_EQ(
        store_landmark_id1,
        vi_map_.getVertex(obs.frame_id.vertex_id).getObservedLandmarkId(obs));
  }
}

}  // namespace map_optimization_legacy

MAPLAB_UNITTEST_ENTRYPOINT
