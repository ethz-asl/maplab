#include <glog/logging.h>
#include <maplab-common/test/testing-entrypoint.h>

#include "map-resources/optional-sensor-resources.h"
#include "map-resources/resource-common.h"

namespace backend {

class OptionalSensorResourcesTest : public ::testing::Test {
 public:
  virtual void SetUp() {
    common::generateId(&resource_id_2_);
    optional_sensor_resource_.resource_id_map().emplace(2, resource_id_2_);

    common::generateId(&resource_id_100_);
    optional_sensor_resource_.resource_id_map().emplace(100, resource_id_100_);

    common::generateId(&resource_id_101_A_);
    optional_sensor_resource_.resource_id_map().emplace(
        101, resource_id_101_A_);

    common::generateId(&resource_id_200_);
    optional_sensor_resource_.resource_id_map().emplace(200, resource_id_200_);

    common::generateId(&resource_id_9_);
    optional_sensor_resource_.resource_id_map().emplace(9, resource_id_9_);

    common::generateId(&resource_id_101_B_);
    optional_sensor_resource_.resource_id_map().emplace(
        101, resource_id_101_B_);
  }

  OptionalSensorResources optional_sensor_resource_;

  backend::ResourceId resource_id_100_;
  backend::ResourceId resource_id_101_A_;
  backend::ResourceId resource_id_101_B_;
  backend::ResourceId resource_id_2_;
  backend::ResourceId resource_id_200_;
  backend::ResourceId resource_id_9_;
};

TEST_F(OptionalSensorResourcesTest, TestGetter) {
  backend::ResourceId resource_id_result;

  EXPECT_TRUE(optional_sensor_resource_.getResourceId(2, &resource_id_result));
  EXPECT_EQ(resource_id_result, resource_id_2_);

  EXPECT_TRUE(optional_sensor_resource_.getResourceId(9, &resource_id_result));
  EXPECT_EQ(resource_id_result, resource_id_9_);

  EXPECT_TRUE(
      optional_sensor_resource_.getResourceId(100, &resource_id_result));
  EXPECT_EQ(resource_id_result, resource_id_100_);

  EXPECT_TRUE(
      optional_sensor_resource_.getResourceId(101, &resource_id_result));
  EXPECT_EQ(resource_id_result, resource_id_101_A_);

  EXPECT_TRUE(
      optional_sensor_resource_.getResourceId(200, &resource_id_result));
  EXPECT_EQ(resource_id_result, resource_id_200_);
}

TEST_F(OptionalSensorResourcesTest, TestDelete) {
  backend::ResourceId resource_id_result;
  EXPECT_TRUE(
      optional_sensor_resource_.getResourceId(101, &resource_id_result));
  EXPECT_EQ(resource_id_result, resource_id_101_A_);

  EXPECT_TRUE(optional_sensor_resource_.deleteResourceId(101));

  EXPECT_FALSE(
      optional_sensor_resource_.getResourceId(101, &resource_id_result));
  EXPECT_FALSE(
      optional_sensor_resource_.getResourceId(101, &resource_id_result));

  EXPECT_FALSE(optional_sensor_resource_.deleteResourceId(101));
}

TEST_F(OptionalSensorResourcesTest, TestGetClosestResourceId) {
  backend::StampedResourceId stamped_resource_id_result;

  constexpr int64_t kToleranceHigh = 2;
  constexpr int64_t kToleranceLow = 1;

  // Near, on, and after the first timestamp.
  EXPECT_TRUE(
      optional_sensor_resource_.getClosestResourceId(
          0, kToleranceHigh, &stamped_resource_id_result));
  EXPECT_EQ(stamped_resource_id_result.second, resource_id_2_);

  EXPECT_FALSE(
      optional_sensor_resource_.getClosestResourceId(
          0, kToleranceLow, &stamped_resource_id_result));

  EXPECT_TRUE(
      optional_sensor_resource_.getClosestResourceId(
          2, kToleranceLow, &stamped_resource_id_result));
  EXPECT_EQ(stamped_resource_id_result.second, resource_id_2_);

  EXPECT_TRUE(
      optional_sensor_resource_.getClosestResourceId(
          3, kToleranceLow, &stamped_resource_id_result));
  EXPECT_EQ(stamped_resource_id_result.second, resource_id_2_);

  // Near, on, and after a "normal" timestamp.
  EXPECT_TRUE(
      optional_sensor_resource_.getClosestResourceId(
          8, kToleranceLow, &stamped_resource_id_result));
  EXPECT_EQ(stamped_resource_id_result.second, resource_id_9_);

  EXPECT_TRUE(
      optional_sensor_resource_.getClosestResourceId(
          9, kToleranceLow, &stamped_resource_id_result));
  EXPECT_EQ(stamped_resource_id_result.second, resource_id_9_);

  EXPECT_TRUE(
      optional_sensor_resource_.getClosestResourceId(
          10, kToleranceLow, &stamped_resource_id_result));
  EXPECT_EQ(stamped_resource_id_result.second, resource_id_9_);

  EXPECT_FALSE(
      optional_sensor_resource_.getClosestResourceId(
          11, kToleranceLow, &stamped_resource_id_result));

  // Near, on, and after identical timestamps.
  EXPECT_TRUE(
      optional_sensor_resource_.getClosestResourceId(
          99, kToleranceLow, &stamped_resource_id_result));
  EXPECT_EQ(stamped_resource_id_result.second, resource_id_100_);

  EXPECT_TRUE(
      optional_sensor_resource_.getClosestResourceId(
          100, kToleranceLow, &stamped_resource_id_result));
  EXPECT_EQ(stamped_resource_id_result.second, resource_id_100_);

  EXPECT_TRUE(
      optional_sensor_resource_.getClosestResourceId(
          101, kToleranceLow, &stamped_resource_id_result));
  EXPECT_EQ(stamped_resource_id_result.second, resource_id_101_A_);

  EXPECT_TRUE(
      optional_sensor_resource_.getClosestResourceId(
          102, kToleranceLow, &stamped_resource_id_result));
  EXPECT_EQ(stamped_resource_id_result.second, resource_id_101_A_);

  // Near, on, and after the last timestamp.
  EXPECT_TRUE(
      optional_sensor_resource_.getClosestResourceId(
          199, kToleranceLow, &stamped_resource_id_result));
  EXPECT_EQ(stamped_resource_id_result.second, resource_id_200_);

  EXPECT_TRUE(
      optional_sensor_resource_.getClosestResourceId(
          200, kToleranceLow, &stamped_resource_id_result));
  EXPECT_EQ(stamped_resource_id_result.second, resource_id_200_);

  EXPECT_TRUE(
      optional_sensor_resource_.getClosestResourceId(
          201, kToleranceLow, &stamped_resource_id_result));
  EXPECT_EQ(stamped_resource_id_result.second, resource_id_200_);

  EXPECT_FALSE(
      optional_sensor_resource_.getClosestResourceId(
          202, kToleranceLow, &stamped_resource_id_result));
}

}  // namespace backend

MAPLAB_UNITTEST_ENTRYPOINT
