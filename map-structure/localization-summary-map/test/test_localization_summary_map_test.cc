#include <memory>

#include <Eigen/Core>
#include <aslam/common/hash-id.h>
#include <maplab-common/pose_types.h>
#include <maplab-common/test/testing-entrypoint.h>
#include <maplab-common/test/testing-predicates.h>
#include <maplab-common/unique-id.h>
#include <posegraph/unique-id.h>
#include <vi-map/test/vi-map-generator.h>
#include <vi-map/unique-id.h>

#include "localization-summary-map/localization-summary-map-creation.h"
#include "localization-summary-map/localization-summary-map.h"

class LocalizationSummaryMapTest : public ::testing::Test {
 protected:
  virtual void SetUp() {
    constructProblem();
  }
  void constructProblem();
  void constructVertexWithVisualNFrame();

  vi_map::VIMap map_;
  vi_map::LandmarkId landmark_1_id_;
  vi_map::LandmarkId landmark_2_id_;
  vi_map::LandmarkId landmark_3_id_;
};

void LocalizationSummaryMapTest::constructProblem() {
  vi_map::VIMapGenerator generator(map_, 42);

  vi_map::MissionId mission_id =
      generator.createMission(pose::Transformation());
  pose_graph::VertexId v1 =
      generator.createVertex(mission_id, pose::Transformation());
  pose_graph::VertexId v2 =
      generator.createVertex(mission_id, pose::Transformation());
  pose_graph::VertexId v3 =
      generator.createVertex(mission_id, pose::Transformation());
  landmark_1_id_ =
      generator.createLandmark(Eigen::Vector3d(0, 0, 1), v1, {v2, v3});
  landmark_2_id_ = generator.createLandmark(Eigen::Vector3d(0, 0, 2), v2, {v3});
  landmark_3_id_ = generator.createLandmark(Eigen::Vector3d(0, 0, 3), v1, {v2});

  generator.generateMap();
}

TEST_F(LocalizationSummaryMapTest, LocalizationSummaryCreationFromMapTest) {
  summary_map::LocalizationSummaryMap summary_map;
  summary_map::LocalizationSummaryMapId id;
  common::generateId(&id);
  summary_map.setId(id);

  vi_map::LandmarkIdList summary_landmark_ids;
  summary_landmark_ids.push_back(landmark_1_id_);
  summary_landmark_ids.push_back(landmark_2_id_);

  summary_map::createLocalizationSummaryMapFromLandmarkList(
      map_, summary_landmark_ids, &summary_map);

  const Eigen::Matrix3Xf& G_landmark_position = summary_map.GLandmarkPosition();

  ASSERT_EQ(G_landmark_position.cols(), 2);
  EXPECT_NEAR_EIGEN(G_landmark_position.col(0), Eigen::Vector3f(0, 0, 1), 1e-9);
  EXPECT_NEAR_EIGEN(G_landmark_position.col(1), Eigen::Vector3f(0, 0, 2), 1e-9);

  EXPECT_EQ(summary_map.projectedDescriptors().cols(), 5);
  const Eigen::Matrix<unsigned int, Eigen::Dynamic, 1>& obs_to_landmark =
      summary_map.observationToLandmarkIndex();
  EXPECT_EQ(5, obs_to_landmark.rows());
  EXPECT_EQ(0, obs_to_landmark(0, 0));
  EXPECT_EQ(0, obs_to_landmark(1, 0));
  EXPECT_EQ(0, obs_to_landmark(2, 0));
  EXPECT_EQ(1, obs_to_landmark(3, 0));
  EXPECT_EQ(1, obs_to_landmark(4, 0));

  const Eigen::Matrix<unsigned int, Eigen::Dynamic, 1>& landmark_observers =
      summary_map.observerIndices();
  EXPECT_EQ(5, landmark_observers.rows());
  EXPECT_EQ(0, landmark_observers(0, 0));
  EXPECT_EQ(1, landmark_observers(1, 0));
  EXPECT_EQ(2, landmark_observers(2, 0));
  EXPECT_EQ(1, landmark_observers(3, 0));
  EXPECT_EQ(2, landmark_observers(4, 0));
}

MAPLAB_UNITTEST_ENTRYPOINT
