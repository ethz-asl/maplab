#include <memory>
#include <unordered_map>
#include <unordered_set>

#include <glog/logging.h>
#include <maplab-common/test/testing-entrypoint.h>
#include <maplab-common/test/testing-predicates.h>
#include <maplab-common/unique-id.h>
#include <vi-map/pose-graph.h>
#include <vi-map/unique-id.h>
#include <vi-map/vi-map.h>

namespace map_optimization_legacy {

class ViwlsGraph : public ::testing::Test {
 protected:
  ViwlsGraph() {}

  virtual void SetUp() {
    cameras_ = aslam::NCamera::createTestNCamera(kNumCameras);

    addVertices();
    addLandmarksAndKeypoints();

    CHECK(!landmark_ids_.empty());
  }

  void addVertices();
  void addLandmarksAndKeypoints();

  vi_map::VIMap vi_map_;
  std::vector<pose_graph::VertexId> vertex_ids_;
  std::vector<vi_map::LandmarkId> landmark_ids_;

  aslam::NCamera::Ptr cameras_;

  static constexpr unsigned int kNumOfVertices = 2;
  static constexpr unsigned int kNumOfLandmarksPerStoreLandmark = 2;
  static constexpr unsigned int kNumOfStoreLandmarks = 2;
  static constexpr unsigned int kNumOfKeypointsPerVertex =
      kNumOfStoreLandmarks * kNumOfLandmarksPerStoreLandmark;
  static constexpr unsigned int kDescriptorBytes = 48;

  static constexpr unsigned int kNumCameras = 1;
  static constexpr unsigned int kVisualFrameIndex = 0;
};

void ViwlsGraph::addVertices() {
  vertex_ids_.clear();
  vi_map::MissionId mission_id;
  generateId(&mission_id);
  for (unsigned int i = 0; i < kNumOfVertices; ++i) {
    pose_graph::VertexId vertex_id;
    vi_map::Vertex::UniquePtr vertex(new vi_map::Vertex(cameras_));
    generateId(&vertex_id);
    vertex->setId(vertex_id);
    vertex->setMissionId(mission_id);

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

    vi_map_.posegraph.addVertex(std::move(vertex));
    vertex_ids_.push_back(vertex_id);
  }
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
}

void ViwlsGraph::addLandmarksAndKeypoints() {
  landmark_ids_.clear();
  for (unsigned int i = 0; i < kNumOfStoreLandmarks; ++i) {
    vi_map::LandmarkId landmark_id;
    common::generateId(&landmark_id);
    landmark_ids_.push_back(landmark_id);

    vi_map::Landmark store_landmark;
    store_landmark.setId(landmark_id);
    CHECK(!vertex_ids_.empty());
    vi_map_.getVertex(vertex_ids_[0])
        .getLandmarks()
        .addLandmark(store_landmark);
    vi_map_.landmark_index.addLandmarkAndVertexReference(
        landmark_id, vertex_ids_[0]);

    for (unsigned int j = 0; j < kNumOfLandmarksPerStoreLandmark; ++j) {
      for (unsigned int k = 0; k < kNumOfVertices; ++k) {
        vi_map_.getVertex(vertex_ids_[k])
            .addObservedLandmarkId(kVisualFrameIndex, landmark_id);
      }
    }
  }
}

TEST_F(ViwlsGraph, LandmarkDeletionTest) {
  const vi_map::LandmarkId& landmark_id = landmark_ids_[0];

  const vi_map::Landmark& landmark = vi_map_.getLandmark(landmark_id);
  std::vector<vi_map::KeypointIdentifier> landmark_observations;
  landmark.forEachObservation(
      [&](const vi_map::KeypointIdentifier& keypoint_id) {
        landmark_observations.push_back(keypoint_id);
      });

  const vi_map::Vertex& vertex = vi_map_.getLandmarkStoreVertex(landmark_id);
  EXPECT_TRUE(vi_map_.hasLandmark(landmark_id));
  EXPECT_TRUE(vertex.hasStoredLandmark(landmark_id));

  vi_map_.removeLandmark(landmark_id);

  EXPECT_FALSE(vi_map_.hasLandmark(landmark_id));
  EXPECT_FALSE(vertex.hasStoredLandmark(landmark_id));
  for (const vi_map::KeypointIdentifier& observation : landmark_observations) {
    const vi_map::Vertex& observer_vertex =
        vi_map_.getVertex(observation.frame_id.vertex_id);
    EXPECT_FALSE(observer_vertex.getObservedLandmarkId(observation).isValid());
  }
}

}  // namespace map_optimization_legacy

MAPLAB_UNITTEST_ENTRYPOINT
