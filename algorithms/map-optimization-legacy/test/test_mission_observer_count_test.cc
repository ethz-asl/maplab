#include <memory>
#include <random>
#include <unordered_map>
#include <unordered_set>

#include <glog/logging.h>
#include <maplab-common/test/testing-entrypoint.h>
#include <maplab-common/test/testing-predicates.h>
#include <maplab-common/unique-id.h>

#include "vi-map/check-map-consistency.h"
#include "vi-map/pose-graph.h"
#include "vi-map/vi-map.h"

namespace map_optimization_legacy {

class ViwlsGraph : public ::testing::Test {
 protected:
  virtual void SetUp() {
    cameras_ = aslam::NCamera::createTestNCamera(kNumCameras);
  }

  void addMissions(
      unsigned int num_missions, unsigned int num_vertices_per_mission);
  void addMission(
      const vi_map::MissionId& mission_id, unsigned int num_vertices);
  void addVisualFrameToVertex(vi_map::Vertex* vertex_ptr);
  void addObservedLandmarkToMissions(
      unsigned int num_of_observer_missions,
      const vi_map::LandmarkId& landmark_id);
  void addObservedLandmarkToMissions(
      const vi_map::MissionIdSet& observer_missions_set,
      const vi_map::LandmarkId& landmark_id,
      const unsigned int num_observations_per_mission);
  void fillVIMap();

  vi_map::VIMap vi_map_;
  pose_graph::VertexIdList vertex_ids_;
  vi_map::MissionIdList mission_ids_;

  typedef std::unordered_map<vi_map::MissionId, pose_graph::VertexIdList>
      MissionVertexMap;
  MissionVertexMap mission_vertices_map_;

  typedef std::unordered_map<vi_map::LandmarkId, vi_map::MissionIdSet>
      LandmarkObserverMissionMap;
  LandmarkObserverMissionMap landmark_observer_mission_map_;

  typedef std::unordered_map<vi_map::LandmarkId, int>
      TrueMissionObserverCountMap;
  TrueMissionObserverCountMap true_mission_observer_count_map_;

  aslam::NCamera::Ptr cameras_;
  std::default_random_engine generator_;

  static constexpr unsigned int kNumOfKeypointsPerVertex = 100;
  static constexpr int kNumCameras = 1;
  static constexpr int kVisualFrameIndex = 0;
  static constexpr unsigned int kRandomSeed = 5;
  static constexpr int kDescriptorBytes = 48;
};

void ViwlsGraph::addMissions(
    unsigned int num_missions, unsigned int num_vertices_per_mission) {
  CHECK_GT(num_vertices_per_mission, 0u);
  for (unsigned int i = 0; i < num_missions; ++i) {
    vi_map::MissionId mission_id;
    generateId(&mission_id);
    addMission(mission_id, num_vertices_per_mission);
  }
}

void ViwlsGraph::addVisualFrameToVertex(vi_map::Vertex* vertex_ptr) {
  CHECK_NOTNULL(vertex_ptr);

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
  vertex_ptr->getVisualNFrame().setFrame(kVisualFrameIndex, frame);

  for (unsigned int i = 0; i < kNumOfKeypointsPerVertex; ++i) {
    vertex_ptr->addObservedLandmarkId(kVisualFrameIndex, vi_map::LandmarkId());
  }
}

void ViwlsGraph::addMission(
    const vi_map::MissionId& mission_id, unsigned int num_vertices) {
  CHECK_GT(num_vertices, 0u);

  vi_map::Vertex::UniquePtr vertex(new vi_map::Vertex(cameras_));
  pose_graph::VertexId vertex_id;
  generateId(&vertex_id);

  vertex->setId(vertex_id);
  vertex->setMissionId(mission_id);
  addVisualFrameToVertex(vertex.get());
  vertex_ids_.push_back(vertex_id);
  mission_vertices_map_[mission_id].push_back(vertex_id);

  vi_map::MissionBaseFrameId baseframe_id;
  generateId(&baseframe_id);
  vi_map::MissionBaseFrame mission_baseframe;
  mission_baseframe.setId(baseframe_id);
  vi_map_.mission_base_frames.emplace(baseframe_id, mission_baseframe);

  vi_map::VIMission::UniquePtr mission(new vi_map::VIMission);
  mission->setId(mission_id);
  mission->setRootVertexId(vertex_id);
  mission->setBaseFrameId(baseframe_id);
  vi_map_.missions.emplace(mission_id, std::move(mission));
  mission_ids_.push_back(mission_id);

  vi_map_.addVertex(std::move(vertex));

  for (unsigned int i = 1; i < num_vertices; ++i) {
    pose_graph::VertexId vertex_id;
    vi_map::Vertex::UniquePtr vertex(new vi_map::Vertex(cameras_));
    generateId(&vertex_id);
    vertex->setId(vertex_id);
    vertex->setMissionId(mission_id);
    addVisualFrameToVertex(vertex.get());
    vertex_ids_.push_back(vertex_id);
    vi_map_.addVertex(std::move(vertex));

    mission_vertices_map_[mission_id].push_back(vertex_id);
  }
}

void ViwlsGraph::addObservedLandmarkToMissions(
    unsigned int num_of_observer_missions,
    const vi_map::LandmarkId& landmark_id) {
  CHECK(!mission_ids_.empty());
  CHECK_LE(num_of_observer_missions, mission_ids_.size());

  vi_map::MissionIdSet observer_missions_set;
  std::uniform_int_distribution<int> distribution(0, mission_ids_.size() - 1);
  while (observer_missions_set.size() < num_of_observer_missions) {
    observer_missions_set.insert(mission_ids_[distribution(generator_)]);
  }
  const unsigned int num_observations_per_mission = 2;
  addObservedLandmarkToMissions(
      observer_missions_set, landmark_id, num_observations_per_mission);
}

void ViwlsGraph::addObservedLandmarkToMissions(
    const vi_map::MissionIdSet& observer_missions_set,
    const vi_map::LandmarkId& landmark_id,
    const unsigned int num_observations_per_mission) {
  CHECK(!mission_ids_.empty());

  vi_map::Landmark store_landmark;
  store_landmark.setId(landmark_id);

  // Store landmark in a randomly selected vertex.
  std::uniform_int_distribution<int> distribution(0, vertex_ids_.size() - 1);
  const pose_graph::VertexId& storing_vertex_id =
      vertex_ids_[distribution(generator_)];
  vi_map_.getVertex(storing_vertex_id)
      .getLandmarks()
      .addLandmark(store_landmark);

  // Add observations.
  for (const vi_map::MissionId& mission_id : observer_missions_set) {
    landmark_observer_mission_map_[landmark_id].insert(mission_id);

    for (unsigned int i = 0; i < num_observations_per_mission; ++i) {
      std::uniform_int_distribution<int> distribution(
          0, mission_vertices_map_[mission_id].size() - 1);
      const pose_graph::VertexId& observer_vertex_id =
          mission_vertices_map_[mission_id][distribution(generator_)];

      // Get new keypoint index by counting the number of already existing
      // valid global landmark IDs stored in the vertex.
      const unsigned int keypoint_index =
          vi_map_.getVertex(observer_vertex_id)
              .numValidObservedLandmarkIds(kVisualFrameIndex);
      vi_map_.getVertex(observer_vertex_id)
          .setObservedLandmarkId(
              kVisualFrameIndex, keypoint_index, landmark_id);
      vi_map_.getVertex(storing_vertex_id)
          .getLandmarks()
          .getLandmark(landmark_id)
          .addObservation(
              observer_vertex_id, kVisualFrameIndex, keypoint_index);
    }
  }

  vi_map_.landmark_index.addLandmarkAndVertexReference(
      landmark_id, storing_vertex_id);
}

void ViwlsGraph::fillVIMap() {
  const unsigned int total_num_of_missions = 10;
  const unsigned int vertices_per_mission = 10;
  addMissions(total_num_of_missions, vertices_per_mission);

  for (unsigned int num_of_observer_missions = 1;
       num_of_observer_missions < total_num_of_missions;
       ++num_of_observer_missions) {
    vi_map::LandmarkId landmark_id;
    generateId(&landmark_id);

    addObservedLandmarkToMissions(num_of_observer_missions, landmark_id);
    true_mission_observer_count_map_.emplace(
        landmark_id, num_of_observer_missions);
  }
}

class ViwlsGraphParametrized
    : public ViwlsGraph,
      public ::testing::WithParamInterface<unsigned int> {};  // NOLINT

// This test verifies if numLandmarkObserverMissions of the VIMap returns
// correct estimation of the number of missions that observe a certain
// landmark.
TEST_F(ViwlsGraph, NumLandmarkObserverMissionsTest) {
  fillVIMap();

  for (const TrueMissionObserverCountMap::value_type& true_observation_count :
       true_mission_observer_count_map_) {
    EXPECT_EQ(
        true_observation_count.second,
        static_cast<int>(
            vi_map_.numLandmarkObserverMissions(true_observation_count.first)));
  }
}

// This test verifies if getLandmarkObserverMissions of the VIMap returns
// returns correct mission IDs that observe a certain landmark.
TEST_F(ViwlsGraph, GetLandmarkObserverMissionsTest) {
  fillVIMap();

  for (const LandmarkObserverMissionMap::value_type&
           true_landmark_observer_missions : landmark_observer_mission_map_) {
    vi_map::MissionIdSet observer_missions;
    vi_map_.getLandmarkObserverMissions(
        true_landmark_observer_missions.first, &observer_missions);
    EXPECT_EQ(true_landmark_observer_missions.second, observer_missions);
  }
}

INSTANTIATE_TEST_CASE_P(
    ViwlsGraph, ViwlsGraphParametrized, ::testing::Range(1u, 11u, 2u));

// This test verifies if numExpectedLandmarkObserverMissions of the VIMap
// works as expected. In this test, no other landmarks are coobserved with
// the landmark in question, so we should exactly get the number of missions
// observing this landmark.
TEST_P(ViwlsGraphParametrized, ExpectedObserverMissionCountOnlyObserversTest) {
  const unsigned int vertices_per_mission = 10;
  // Let's add two times more missions than observer ones.
  addMissions(2 * GetParam(), vertices_per_mission);

  vi_map::LandmarkId landmark_id;
  generateId(&landmark_id);
  addObservedLandmarkToMissions(GetParam(), landmark_id);

  EXPECT_EQ(
      GetParam(), vi_map_.numExpectedLandmarkObserverMissions(landmark_id));
}

// This test verifies if numExpectedLandmarkObserverMissions of the VIMap
// works as expected. In this test, a few landmarks are coobserved together
// with the landmark in question in the same frame. Test parameter denotes
// number of missions the coobserved landmarks appear in, so also expected
// observer mission count.
TEST_P(ViwlsGraphParametrized, ExpectedObserverMissionCountContextSearchTest) {
  fillVIMap();

  const unsigned int num_map_missions = mission_ids_.size();

  // A mission with single vertex that will contain landmark in question.
  vi_map::MissionId mission_id;
  generateId(&mission_id);
  addMission(mission_id, 1);

  vi_map::MissionIdSet observer_missions_set;
  observer_missions_set.insert(mission_id);

  // Add a landmark that we want to calculate expectedObserverMissionCount for.
  vi_map::LandmarkId landmark_id;
  generateId(&landmark_id);
  addObservedLandmarkToMissions(observer_missions_set, landmark_id, 3);

  // Selected missions from the map missions.
  std::uniform_int_distribution<int> distribution(0, num_map_missions - 1);
  while (observer_missions_set.size() < GetParam()) {
    observer_missions_set.insert(mission_ids_[distribution(generator_)]);
  }

  // Add landmarks that will be used as context landmarks during expected
  // observer mission estimation.
  const unsigned int num_coobserved_landmarks = 5;
  const unsigned int num_observations_per_mission = 5;
  for (unsigned int i = 0; i < num_coobserved_landmarks; ++i) {
    vi_map::LandmarkId landmark_id;
    generateId(&landmark_id);
    addObservedLandmarkToMissions(
        observer_missions_set, landmark_id, num_observations_per_mission);
  }

  EXPECT_EQ(
      GetParam(), vi_map_.numExpectedLandmarkObserverMissions(landmark_id));
}

}  // namespace map_optimization_legacy

MAPLAB_UNITTEST_ENTRYPOINT
