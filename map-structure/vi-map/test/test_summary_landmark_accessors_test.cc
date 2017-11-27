#include <vector>

#include <gtest/gtest.h>
#include <maplab-common/test/testing-entrypoint.h>
#include <maplab-common/test/testing-predicates.h>

#include "vi-map/localization-summary-map.h"
#include "vi-map/test/vi-map-generator.h"
#include "vi-map/vi-map.h"

namespace vi_map {

class VIMapSummaryLandmarkAccessorsTest : public ::testing::Test {
 protected:
  VIMapSummaryLandmarkAccessorsTest() : map_(), generator_(map_, 42) {}

  virtual void SetUp() {
    pose::Transformation T_G_V;
    pose::Transformation T_G_M;

    vi_map::MissionId mission_id = generator_.createMission(T_G_M);
    vertex_id_ = generator_.createVertex(mission_id, T_G_V);

    map_landmark_p_G_fi_ << 1, 2, 3;
    summary_landmark_p_G_fi_ << 4, 5, 6;
  }

  void addLandmark() {
    pose_graph::VertexIdList empty_vertex_id_list;
    store_landmark_id_ = generator_.createLandmark(
        map_landmark_p_G_fi_, vertex_id_, empty_vertex_id_list);
  }

  void generateMap() {
    generator_.generateMap();
  }

  void addLocalizationSummaryMap() {
    vi_map::LocalizationSummaryMap::Ptr loc_summary_map(
        new vi_map::LocalizationSummaryMap);

    vi_map::LocalizationSummaryMapId id;
    generateId(&id);
    loc_summary_map->setId(id);

    Eigen::Matrix3Xd G_landmark_positions;
    G_landmark_positions.resize(Eigen::NoChange, 1);
    G_landmark_positions = summary_landmark_p_G_fi_;
    loc_summary_map->setGLandmarkPosition(G_landmark_positions);

    // Retrieve generated global landmark id.
    vi_map::GlobalLandmarkIdList summary_global_landmark_ids;
    loc_summary_map->getAllLandmarkIds(&summary_global_landmark_ids);
    ASSERT_FALSE(summary_global_landmark_ids.empty());

    summary_global_landmark_id_ = summary_global_landmark_ids[0];

    map_.addLocalizationSummaryMap(loc_summary_map);
  }

  vi_map::StoreLandmarkId store_landmark_id_;
  Eigen::Vector3d map_landmark_p_G_fi_;

  vi_map::GlobalLandmarkId summary_global_landmark_id_;
  Eigen::Vector3d summary_landmark_p_G_fi_;

  pose_graph::VertexId vertex_id_;

  VIMap map_;
  VIMapGenerator generator_;

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

TEST_F(VIMapSummaryLandmarkAccessorsTest, MapLandmark_p_G_fi) {
  addLandmark();
  generateMap();
  addLocalizationSummaryMap();

  vi_map::GlobalLandmarkIdList global_landmarks;
  map_.getGlobalLandmarksReferencingGivenStoreLandmark(
      store_landmark_id_, &global_landmarks);
  ASSERT_FALSE(global_landmarks.empty());

  EXPECT_NEAR_EIGEN(
      map_landmark_p_G_fi_, map_.getLandmark_G_p_fi(global_landmarks[0]), 1e-6);
}

TEST_F(VIMapSummaryLandmarkAccessorsTest, SummaryLandmark_p_G_fi) {
  addLandmark();
  generateMap();
  addLocalizationSummaryMap();

  EXPECT_NEAR_EIGEN(
      summary_landmark_p_G_fi_,
      map_.getLandmark_G_p_fi(summary_global_landmark_id_), 1e-6);
}

TEST_F(VIMapSummaryLandmarkAccessorsTest, NoMapLandmarkSummaryLandmark_p_G_fi) {
  generateMap();
  addLocalizationSummaryMap();

  EXPECT_NEAR_EIGEN(
      summary_landmark_p_G_fi_,
      map_.getLandmark_G_p_fi(summary_global_landmark_id_), 1e-6);
}

TEST_F(VIMapSummaryLandmarkAccessorsTest, NonexistentLandmark_p_G_fi) {
  addLandmark();
  generateMap();
  addLocalizationSummaryMap();

  vi_map::GlobalLandmarkId nonexistent_landmark_id;
  generateId(&nonexistent_landmark_id);
  EXPECT_DEATH(map_.getLandmark_G_p_fi(nonexistent_landmark_id), "Landmark");
}

TEST_F(VIMapSummaryLandmarkAccessorsTest, GetMapLandmark) {
  addLandmark();
  generateMap();
  addLocalizationSummaryMap();

  vi_map::GlobalLandmarkIdList global_landmarks;
  map_.getGlobalLandmarksReferencingGivenStoreLandmark(
      store_landmark_id_, &global_landmarks);
  ASSERT_FALSE(global_landmarks.empty());

  EXPECT_EQ(store_landmark_id_, map_.getLandmark(global_landmarks[0]).id());
}

TEST_F(VIMapSummaryLandmarkAccessorsTest, GetSummaryLandmark) {
  addLandmark();
  generateMap();
  addLocalizationSummaryMap();

  EXPECT_NEAR_EIGEN(
      summary_landmark_p_G_fi_,
      map_.getLandmark(summary_global_landmark_id_).get_p_B(), 1e-6);
}

TEST_F(VIMapSummaryLandmarkAccessorsTest, GetNonexistentLandmark) {
  addLandmark();
  generateMap();
  addLocalizationSummaryMap();

  vi_map::GlobalLandmarkId nonexistent_landmark_id;
  generateId(&nonexistent_landmark_id);
  EXPECT_DEATH(map_.getLandmark(nonexistent_landmark_id), "Landmark");
}

}  // namespace vi_map

MAPLAB_UNITTEST_ENTRYPOINT
