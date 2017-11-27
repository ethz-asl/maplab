#include <Eigen/Core>
#include <gtest/gtest.h>
#include <maplab-common/test/testing-entrypoint.h>

#include "vi-map/spatial-database.h"
#include "vi-map/test/vi-map-generator.h"

namespace vi_map {

template <typename ObjectIdType>
class SpatialDatabaseTest : public ::testing::Test {
 public:
  SpatialDatabaseTest() : map_(), generator_(map_, 42) {}
  void generateMissions() {
    T_G_X_[0].getPosition() << -.5, -.5, -.5;

    T_G_X_[1].getPosition() << 0, 0, 0;
    T_G_X_[2].getPosition() << 1, 0, 0;
    T_G_X_[3].getPosition() << 2, 0, 0;
    T_G_X_[4].getPosition() << 0, 1, 0;
    T_G_X_[5].getPosition() << 1, 1, 0;
    T_G_X_[6].getPosition() << 2, 1, 0;
    T_G_X_[7].getPosition() << 0, 1, 1;
    T_G_X_[8].getPosition() << 1, 1, 1;
    T_G_X_[9].getPosition() << 2, 1, 1;
    T_G_X_[10].getPosition() << 0, 0, 1;
    T_G_X_[11].getPosition() << 1, 0, 1;
    T_G_X_[12].getPosition() << 2, 0, 1;
    T_G_X_[13].getPosition() << .5, .5, .5;

    T_G_X_[14].getPosition() << 2.5, 1.5, 1.5;

    alternate_T_G_X_.getPosition() << 1, 1, 1;

    T_G_M_.getPosition() << 1, 2, 3;
    alternate_T_G_M_.getPosition() << 2, 3, 4;

    M_ = generator_.createMission(T_G_M_);
    alternate_M_ = generator_.createMission(alternate_T_G_M_);

    createObjects();
    createAlternateMissionObjects();

    generator_.generateMap();

    xyz_resolution_ << 3, 2, 2;
  }

  void createObjects();
  void createAlternateMissionObjects();

 protected:
  VIMap map_;
  VIMapGenerator generator_;
  Eigen::Vector3i xyz_resolution_;
  vi_map::MissionId M_, alternate_M_;
  static constexpr int kNumObjects = 15;
  pose_graph::VertexId V_[kNumObjects], alternate_V_;
  pose::Transformation T_G_X_[kNumObjects], alternate_T_G_X_;
  pose::Transformation T_G_V_;
  pose::Transformation T_G_M_, alternate_T_G_M_;
};

template <>
void SpatialDatabaseTest<pose_graph::VertexId>::createObjects() {
  V_[0] = generator_.createVertex(M_, T_G_X_[0]);
  for (int i = 1; i < kNumObjects; ++i) {
    V_[i] = generator_.createVertex(M_, T_G_X_[i]);
  }
};

template <>
void SpatialDatabaseTest<StoreLandmarkId>::createObjects() {
  T_G_V_.getPosition() << 0, 0, -10;
  V_[0] = generator_.createVertex(M_, T_G_V_);
  for (int i = 0; i < kNumObjects; ++i) {
    generator_.createLandmark(T_G_X_[i].getPosition(), V_[0], {});
  }
};

template <>
void SpatialDatabaseTest<
    pose_graph::VertexId>::createAlternateMissionObjects() {
  alternate_V_ = generator_.createVertex(alternate_M_, alternate_T_G_X_);
};

template <>
void SpatialDatabaseTest<StoreLandmarkId>::createAlternateMissionObjects() {
  T_G_V_.getPosition() << 0, 0, -10;
  V_[0] = generator_.createVertex(alternate_M_, T_G_V_);
  generator_.createLandmark(alternate_T_G_X_.getPosition(), V_[0], {});
};

typedef ::testing::Types<pose_graph::VertexId, StoreLandmarkId> TestTypes;
TYPED_TEST_CASE(SpatialDatabaseTest, TestTypes);

TYPED_TEST(SpatialDatabaseTest, GetObjectsOfGridIndex) {
  this->generateMissions();
  vi_map::SpatialDatabase<TypeParam> spatial_db(
      this->map_, this->xyz_resolution_);
  std::vector<TypeParam> grid_unit_object_ids;
  Eigen::Vector3i grid_unit(2, 1, 1);
  spatial_db.getObjectIdsOfGridIndex(grid_unit, &grid_unit_object_ids);
  EXPECT_EQ(2u, grid_unit_object_ids.size());
  grid_unit << 1, 1, 1;
  spatial_db.getObjectIdsOfGridIndex(grid_unit, &grid_unit_object_ids);
  EXPECT_EQ(3u, grid_unit_object_ids.size());
  grid_unit << 0, 0, 0;
  spatial_db.getObjectIdsOfGridIndex(grid_unit, &grid_unit_object_ids);
  EXPECT_EQ(2u, grid_unit_object_ids.size());
}

TYPED_TEST(SpatialDatabaseTest, GenerateDatabase) {
  this->generateMissions();
  vi_map::SpatialDatabase<TypeParam> spatial_db(
      this->map_, this->xyz_resolution_);
  std::vector<TypeParam> all_map_objects;
  this->map_.getAllIds(&all_map_objects);
  bool all_vertices_are_in_spatial_database = true;
  for (const TypeParam& object_id : all_map_objects) {
    Eigen::Vector3i grid_unit;
    spatial_db.getGridIndexForPosition(
        this->map_.get_p_G(object_id), &grid_unit);
    std::vector<TypeParam> grid_unit_object_ids;
    spatial_db.getObjectIdsOfGridIndex(grid_unit, &grid_unit_object_ids);
    if (std::find(
            grid_unit_object_ids.begin(), grid_unit_object_ids.end(),
            object_id) == grid_unit_object_ids.end()) {
      all_vertices_are_in_spatial_database = false;
    }
  }
  EXPECT_TRUE(all_vertices_are_in_spatial_database);
}

TYPED_TEST(SpatialDatabaseTest, getGridIndex) {
  this->generateMissions();
  vi_map::SpatialDatabase<TypeParam> spatial_db(
      this->map_, this->xyz_resolution_);
  Eigen::Vector3i grid_unit_result, grid_unit_reference(0, 1, 0);
  Eigen::Vector3d p_G_I(.4, .8, .2);
  spatial_db.getGridIndexForPosition(p_G_I, &grid_unit_result);
  EXPECT_EQ(grid_unit_reference, grid_unit_result);

  // Check for grid corner that is furthest of origin (if inside of grid).
  p_G_I << 2.5, 1.5, 1.5;
  grid_unit_reference << 2, 1, 1;
  spatial_db.getGridIndexForPosition(p_G_I, &grid_unit_result);
  EXPECT_EQ(grid_unit_reference, grid_unit_result);
}

TYPED_TEST(SpatialDatabaseTest, getGridResolution) {
  this->generateMissions();
  Eigen::Vector3i grid_resolution_result, grid_resolution_reference(2, 3, 4);
  vi_map::SpatialDatabase<TypeParam> spatial_db(
      this->map_, grid_resolution_reference);
  spatial_db.getGridResolution(&grid_resolution_result);
  EXPECT_EQ(grid_resolution_reference, grid_resolution_result);
}

TYPED_TEST(SpatialDatabaseTest, getNeighborObject) {
  this->generateMissions();
  vi_map::SpatialDatabase<TypeParam> spatial_db(
      this->map_, this->xyz_resolution_);
  std::unordered_set<TypeParam> neighbor_object_ids;
  Eigen::Vector3d p_G_I(0, 0, 0);
  spatial_db.getObjectIdsInRadius(p_G_I, 0.87, &neighbor_object_ids);
  EXPECT_EQ(3u, neighbor_object_ids.size());
}

TYPED_TEST(SpatialDatabaseTest, getStandardMissionNeighborObject) {
  this->generateMissions();
  vi_map::SpatialDatabase<TypeParam> spatial_db(
      this->map_, this->xyz_resolution_);
  std::unordered_set<TypeParam> neighbor_object_ids;
  Eigen::Vector3d p_G_I(.5, .5, .5);
  spatial_db.getObjectIdsOfMissionInRadius(
      this->M_, p_G_I, 0.87, &neighbor_object_ids);
  EXPECT_EQ(9u, neighbor_object_ids.size());
}

TYPED_TEST(SpatialDatabaseTest, getAlternateMissionNeighborObject) {
  this->generateMissions();
  vi_map::SpatialDatabase<TypeParam> spatial_db(
      this->map_, this->xyz_resolution_);
  std::unordered_set<TypeParam> neighbor_object_ids;
  Eigen::Vector3d p_G_I(.5, .5, .5);
  spatial_db.getObjectIdsOfMissionInRadius(
      this->alternate_M_, p_G_I, 0.87, &neighbor_object_ids);
  EXPECT_EQ(1u, neighbor_object_ids.size());
}

TYPED_TEST(SpatialDatabaseTest, getCuboidObjects) {
  Eigen::Vector3d p_G_min(-1, -1, -1), p_G_max(1.1, 1.1, 1.1);
  this->generateMissions();
  vi_map::SpatialDatabase<TypeParam> spatial_db(
      this->map_, this->xyz_resolution_);
  std::vector<TypeParam> cuboid_object_ids;
  spatial_db.getObjectIdsInCuboid(p_G_min, p_G_max, &cuboid_object_ids);
  EXPECT_EQ(11u, cuboid_object_ids.size());
}

TYPED_TEST(SpatialDatabaseTest, getStandardMissionCuboidObjects) {
  Eigen::Vector3d p_G_min(-1, -1, -1), p_G_max(1.1, 1.1, 1.1);
  this->generateMissions();
  vi_map::SpatialDatabase<TypeParam> spatial_db(
      this->map_, this->xyz_resolution_);
  std::vector<TypeParam> cuboid_object_ids;
  spatial_db.getObjectIdsOfMissionInCuboid(
      this->M_, p_G_min, p_G_max, &cuboid_object_ids);
  EXPECT_EQ(10u, cuboid_object_ids.size());
}

TYPED_TEST(SpatialDatabaseTest, getAlternateMissionCuboidObjects) {
  Eigen::Vector3d p_G_min(-1, -1, -1), p_G_max(1.1, 1.1, 1.1);
  this->generateMissions();
  vi_map::SpatialDatabase<TypeParam> spatial_db(
      this->map_, this->xyz_resolution_);
  std::vector<TypeParam> cuboid_object_ids;
  spatial_db.getObjectIdsOfMissionInCuboid(
      this->alternate_M_, p_G_min, p_G_max, &cuboid_object_ids);
  EXPECT_EQ(1u, cuboid_object_ids.size());
}

TYPED_TEST(SpatialDatabaseTest, getNoNeighborObject) {
  this->generateMissions();
  vi_map::SpatialDatabase<TypeParam> spatial_db(
      this->map_, this->xyz_resolution_);
  std::unordered_set<TypeParam> neighbor_object_ids;
  Eigen::Vector3d p_G_I(5, 5, 5);
  spatial_db.getObjectIdsInRadius(p_G_I, 1, &neighbor_object_ids);
  EXPECT_EQ(0u, neighbor_object_ids.size());
}

TYPED_TEST(SpatialDatabaseTest, EmptyMapSpatialDatabase) {
  this->xyz_resolution_ << 3, 3, 3;
  this->generator_.generateMap();
  EXPECT_DEATH(
      SpatialDatabase<TypeParam> spatial_db(this->map_, this->xyz_resolution_),
      "");
}
TYPED_TEST(SpatialDatabaseTest, DistanceToGridCell) {
  this->generateMissions();
  vi_map::SpatialDatabase<TypeParam> spatial_db(
      this->map_, this->xyz_resolution_);
  Eigen::Vector3i grid_unit_reference(1, 1, 1);
  Eigen::Vector3d p_G_0(1, 1, 1);
  EXPECT_DOUBLE_EQ(
      .0, spatial_db.getSquaredDistanceToGridCell(p_G_0, grid_unit_reference));

  p_G_0 << 0, 1, 1;
  EXPECT_DOUBLE_EQ(
      .25, spatial_db.getSquaredDistanceToGridCell(p_G_0, grid_unit_reference));

  p_G_0 << 2, 1, 1;
  EXPECT_DOUBLE_EQ(
      .25, spatial_db.getSquaredDistanceToGridCell(p_G_0, grid_unit_reference));

  p_G_0 << 1, 0, 1;
  EXPECT_DOUBLE_EQ(
      .25, spatial_db.getSquaredDistanceToGridCell(p_G_0, grid_unit_reference));

  p_G_0 << 1, 2, 1;
  EXPECT_DOUBLE_EQ(
      .25, spatial_db.getSquaredDistanceToGridCell(p_G_0, grid_unit_reference));

  p_G_0 << 1, 1, 0;
  EXPECT_DOUBLE_EQ(
      .25, spatial_db.getSquaredDistanceToGridCell(p_G_0, grid_unit_reference));

  p_G_0 << 1, 1, 2;
  EXPECT_DOUBLE_EQ(
      .25, spatial_db.getSquaredDistanceToGridCell(p_G_0, grid_unit_reference));

  // Check distance to corners from point that is diagonally sqrt(3) units away.
  std::vector<double> x_coord({-1.5, 1.5});
  std::vector<double> y_coord({-1.5, 1.5});
  std::vector<double> z_coord({-1.5, 1.5});
  for (double x : x_coord) {
    for (double y : y_coord) {
      for (double z : z_coord) {
        p_G_0 << 1 + x, 1 + y, 1 + z;
        EXPECT_DOUBLE_EQ(
            3., spatial_db.getSquaredDistanceToGridCell(
                    p_G_0, grid_unit_reference));
      }
    }
  }
}
}  // namespace vi_map

MAPLAB_UNITTEST_ENTRYPOINT
