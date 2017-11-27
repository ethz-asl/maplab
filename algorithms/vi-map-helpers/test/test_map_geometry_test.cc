#include <Eigen/Core>
#include <gtest/gtest.h>
#include <maplab-common/test/testing-entrypoint.h>
#include <vi-map/test/vi-map-generator.h>

#include "vi-map-helpers/vi-map-geometry.h"

namespace vi_map_helpers {

template <typename ObjectIdType>
class MapBoundingBoxTest : public ::testing::Test {
 protected:
  MapBoundingBoxTest() : map_(), map_geometry_(map_), generator_(map_, 42) {}

  void generateMissions() {
    T_G_X_[0].getPosition() << 1, 2, 3;
    T_G_X_[1].getPosition() << 4, 5, 6;
    T_G_X_[2].getPosition() << 7, 8, 9;

    T_G_X_[3].getPosition() << -1, -2, -3;
    T_G_X_[4].getPosition() << -4, -5, -6;
    T_G_X_[5].getPosition() << -7, -8, -9;

    T_G_M_[0].getPosition() << .1, .1, .1;
    T_G_M_[1].getPosition() << .5, .5, .5;
    T_G_M_[2].getPosition() << 1, 2, 2;

    for (int i = 0; i < 3; ++i) {
      // Landmarks have to be visible for vertex.
      T_G_V_[i].getPosition() << 0, 0, -20;
    }

    M_[0] = generator_.createMission(T_G_M_[0]);
    M_[1] = generator_.createMission(T_G_M_[1]);
    M_[2] = generator_.createMission(T_G_M_[2]);

    createBoxData();

    generator_.generateMap();
  }

  void createBoxData();

  vi_map::VIMap map_;
  VIMapGeometry map_geometry_;
  vi_map::VIMapGenerator generator_;
  vi_map::MissionId M_[3];
  pose_graph::VertexId V_[7];
  pose::Transformation T_G_X_[7];
  pose::Transformation T_G_M_[3];
  pose::Transformation T_G_V_[3];

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

template <>
void MapBoundingBoxTest<pose_graph::VertexId>::createBoxData() {
  V_[0] = generator_.createVertex(M_[0], T_G_X_[0]);
  V_[1] = generator_.createVertex(M_[0], T_G_X_[1]);
  V_[2] = generator_.createVertex(M_[0], T_G_X_[2]);

  V_[3] = generator_.createVertex(M_[1], T_G_X_[3]);
  V_[4] = generator_.createVertex(M_[1], T_G_X_[4]);
  V_[5] = generator_.createVertex(M_[1], T_G_X_[5]);

  V_[6] = generator_.createVertex(M_[2], pose::Transformation());
}

template <>
void MapBoundingBoxTest<vi_map::LandmarkId>::createBoxData() {
  for (int i = 0; i < 3; ++i) {
    V_[i] = generator_.createVertex(M_[i], T_G_V_[i]);
  }

  for (int i = 0; i < 6; ++i) {
    generator_.createLandmark(T_G_X_[i].getPosition(), V_[i / 3], {});
  }

  generator_.createLandmark(Eigen::Vector3d(0, 0, 0), V_[2], {});
}

typedef ::testing::Types<pose_graph::VertexId, vi_map::LandmarkId>
    TestTypes;
TYPED_TEST_CASE(MapBoundingBoxTest, TestTypes);

TYPED_TEST(MapBoundingBoxTest, MissionBoundingBoxTest) {
  Eigen::Vector3d xyz_max, xyz_min;
  this->generateMissions();
  this->map_geometry_.template getBoundingBox<TypeParam>(
      this->M_[0], &xyz_min, &xyz_max);
  EXPECT_DOUBLE_EQ(7, xyz_max[0]);
  EXPECT_DOUBLE_EQ(8, xyz_max[1]);
  EXPECT_DOUBLE_EQ(9, xyz_max[2]);
  EXPECT_DOUBLE_EQ(1, xyz_min[0]);
  EXPECT_DOUBLE_EQ(2, xyz_min[1]);
  EXPECT_DOUBLE_EQ(3, xyz_min[2]);
}

TYPED_TEST(MapBoundingBoxTest, SingleVertexMissionBoundingBoxTest) {
  Eigen::Vector3d xyz_max, xyz_min;
  this->generateMissions();
  this->map_geometry_.template getBoundingBox<TypeParam>(
      this->M_[2], &xyz_min, &xyz_max);
  EXPECT_DOUBLE_EQ(0, xyz_max[0]);
  EXPECT_DOUBLE_EQ(0, xyz_max[1]);
  EXPECT_DOUBLE_EQ(0, xyz_max[2]);
  EXPECT_DOUBLE_EQ(0, xyz_min[0]);
  EXPECT_DOUBLE_EQ(0, xyz_min[1]);
  EXPECT_DOUBLE_EQ(0, xyz_min[2]);
}

TYPED_TEST(MapBoundingBoxTest, MapBoundingBoxTest) {
  Eigen::Vector3d xyz_max, xyz_min;
  this->generateMissions();
  this->map_geometry_.template getBoundingBox<TypeParam>(&xyz_min, &xyz_max);
  EXPECT_DOUBLE_EQ(7, xyz_max[0]);
  EXPECT_DOUBLE_EQ(8, xyz_max[1]);
  EXPECT_DOUBLE_EQ(9, xyz_max[2]);
  EXPECT_DOUBLE_EQ(-7, xyz_min[0]);
  EXPECT_DOUBLE_EQ(-8, xyz_min[1]);
  EXPECT_DOUBLE_EQ(-9, xyz_min[2]);
}

TYPED_TEST(MapBoundingBoxTest, EmptyMapBoundingBoxTest) {
  Eigen::Vector3d xyz_max, xyz_min;
  this->generator_.generateMap();
  EXPECT_DEATH(
      this->map_geometry_.template getBoundingBox<TypeParam>(
          &xyz_min, &xyz_max),
      "");
}

TYPED_TEST(MapBoundingBoxTest, EmptyMissionBoundingBoxTest) {
  Eigen::Vector3d xyz_max, xyz_min;
  this->generator_.generateMap();
  vi_map::MissionId id;
  generateId(&id);
  EXPECT_DEATH(
      this->map_geometry_.template getBoundingBox<TypeParam>(
          id, &xyz_min, &xyz_max),
      "");
}

}  // namespace vi_map_helpers

MAPLAB_UNITTEST_ENTRYPOINT
