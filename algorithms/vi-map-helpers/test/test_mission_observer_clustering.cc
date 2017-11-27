#include <Eigen/Core>
#include <gtest/gtest.h>
#include <maplab-common/test/testing-entrypoint.h>
#include <vi-map/test/vi-map-generator.h>

#include "vi-map-helpers/vi-map-queries.h"

namespace vi_map_helpers {

class VertexClusteringTest : public ::testing::Test {
 private:
  static constexpr size_t kNumVertices = 7u;
  static constexpr size_t kNumLandmarks = 4u;

 protected:
  VertexClusteringTest() : map_(), map_queries_(map_), generator_(map_, 42) {}

  virtual void SetUp() {
    M_ = generator_.createMission(T_G_M_);

    for (size_t i = 0; i < kNumVertices; ++i) {
      V_[i] = generator_.createVertex(M_, T_G_V_[i]);
    }

    addLandmarksForObserverClusteringTest();

    generator_.generateMap();
  }

  void addLandmarksForObserverClusteringTest();

  vi_map::VIMap map_;
  VIMapQueries map_queries_;
  vi_map::VIMapGenerator generator_;
  vi_map::MissionId M_;
  pose_graph::VertexId V_[kNumVertices];
  pose::Transformation T_G_M_;
  pose::Transformation T_G_V_[kNumVertices];
  pose::Transformation T_G_L_[kNumLandmarks];

  // Landmark with one observing vertex.
  vi_map::LandmarkId one_observer_landmark_;

  // Landmark with single cluster.
  vi_map::LandmarkId one_observer_cluster_landmark_;

  // Landmark with two clusters.
  vi_map::LandmarkId two_observer_clusters_landmark_;

  // Landmark with three clusters.
  vi_map::LandmarkId three_observer_clusters_landmark_;

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

void VertexClusteringTest::addLandmarksForObserverClusteringTest() {
  for (size_t i = 0; i < kNumLandmarks; ++i) {
    // Put the landmarks in front of the camera.
    T_G_L_[i].getPosition() << 0, 0, 1;
  }

  // Landmark with one observing vertex.
  one_observer_landmark_ =
      generator_.createLandmark(T_G_L_[0].getPosition(), V_[0], {});

  // Landmark with single cluster.
  one_observer_cluster_landmark_ =
      generator_.createLandmark(T_G_L_[1].getPosition(), V_[4], {V_[3], V_[5]});

  // Landmark with two clusters.
  two_observer_clusters_landmark_ = generator_.createLandmark(
      T_G_L_[2].getPosition(), V_[6], {V_[2], V_[1], V_[5], V_[4]});

  // Landmark with three clusters.
  three_observer_clusters_landmark_ = generator_.createLandmark(
      T_G_L_[3].getPosition(), V_[6], {V_[2], V_[0], V_[3], V_[4]});
}

TEST_F(VertexClusteringTest, NonexistentLandmark) {
  vi_map::LandmarkId nonexistent_landmark_id;
  common::generateId(&nonexistent_landmark_id);

  std::vector<pose_graph::VertexIdSet> clustered_vertices;
  EXPECT_DEATH(
      map_queries_.getConsecutiveLandmarkObserverGroupsFromMission(
          nonexistent_landmark_id, M_, &clustered_vertices),
      "Check");
}

TEST_F(VertexClusteringTest, OneObserverLandmark) {
  std::vector<pose_graph::VertexIdSet> clustered_vertices;
  map_queries_.getConsecutiveLandmarkObserverGroupsFromMission(
      one_observer_landmark_, M_, &clustered_vertices);

  ASSERT_EQ(1u, clustered_vertices.size());
  CHECK_EQ(clustered_vertices[0].count(V_[0]), 1u);
}

TEST_F(VertexClusteringTest, OneClusterLandmark) {
  std::vector<pose_graph::VertexIdSet> clustered_vertices;
  map_queries_.getConsecutiveLandmarkObserverGroupsFromMission(
      one_observer_cluster_landmark_, M_, &clustered_vertices);

  ASSERT_EQ(1u, clustered_vertices.size());
  CHECK_EQ(clustered_vertices[0].count(V_[3]), 1u);
  CHECK_EQ(clustered_vertices[0].count(V_[4]), 1u);
  CHECK_EQ(clustered_vertices[0].count(V_[5]), 1u);
}

TEST_F(VertexClusteringTest, TwoClustersLandmark) {
  std::vector<pose_graph::VertexIdSet> clustered_vertices;
  map_queries_.getConsecutiveLandmarkObserverGroupsFromMission(
      two_observer_clusters_landmark_, M_, &clustered_vertices);

  ASSERT_EQ(2u, clustered_vertices.size());

  CHECK_EQ(clustered_vertices[0].count(V_[1]), 1u);
  CHECK_EQ(clustered_vertices[0].count(V_[2]), 1u);
  CHECK_EQ(clustered_vertices[1].count(V_[4]), 1u);
  CHECK_EQ(clustered_vertices[1].count(V_[5]), 1u);
  CHECK_EQ(clustered_vertices[1].count(V_[6]), 1u);
}

TEST_F(VertexClusteringTest, ThreeClustersLandmark) {
  std::vector<pose_graph::VertexIdSet> clustered_vertices;
  map_queries_.getConsecutiveLandmarkObserverGroupsFromMission(
      three_observer_clusters_landmark_, M_, &clustered_vertices);

  ASSERT_EQ(3u, clustered_vertices.size());

  CHECK_EQ(clustered_vertices[0].count(V_[0]), 1u);
  CHECK_EQ(clustered_vertices[1].count(V_[2]), 1u);
  CHECK_EQ(clustered_vertices[1].count(V_[3]), 1u);
  CHECK_EQ(clustered_vertices[1].count(V_[4]), 1u);
  CHECK_EQ(clustered_vertices[2].count(V_[6]), 1u);
}

}  // namespace vi_map_helpers

MAPLAB_UNITTEST_ENTRYPOINT
