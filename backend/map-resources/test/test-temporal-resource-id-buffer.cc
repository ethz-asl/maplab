#include <glog/logging.h>
#include <maplab-common/test/testing-entrypoint.h>

#include "map-resources/resource-common.h"
#include "map-resources/temporal-resource-id-buffer.h"

namespace backend {

class TemporalResourceIdBufferTest : public ::testing::Test {
 public:
  virtual void SetUp() {
    aslam::generateId(&resource_id_2_);
    resource_id_buffer_.resource_id_map().emplace(2, resource_id_2_);

    aslam::generateId(&resource_id_100_);
    resource_id_buffer_.resource_id_map().emplace(100, resource_id_100_);

    aslam::generateId(&resource_id_101_A_);
    resource_id_buffer_.resource_id_map().emplace(101, resource_id_101_A_);

    aslam::generateId(&resource_id_200_);
    resource_id_buffer_.resource_id_map().emplace(200, resource_id_200_);

    aslam::generateId(&resource_id_9_);
    resource_id_buffer_.resource_id_map().emplace(9, resource_id_9_);

    aslam::generateId(&resource_id_101_B_);
    resource_id_buffer_.resource_id_map().emplace(101, resource_id_101_B_);
  }

  TemporalResourceIdBuffer resource_id_buffer_;

  backend::ResourceId resource_id_100_;
  backend::ResourceId resource_id_101_A_;
  backend::ResourceId resource_id_101_B_;
  backend::ResourceId resource_id_2_;
  backend::ResourceId resource_id_200_;
  backend::ResourceId resource_id_9_;
};

TEST_F(TemporalResourceIdBufferTest, TestGetter) {
  backend::ResourceId resource_id_result;

  EXPECT_TRUE(resource_id_buffer_.getResourceId(2, &resource_id_result));
  EXPECT_EQ(resource_id_result, resource_id_2_);

  EXPECT_TRUE(resource_id_buffer_.getResourceId(9, &resource_id_result));
  EXPECT_EQ(resource_id_result, resource_id_9_);

  EXPECT_TRUE(resource_id_buffer_.getResourceId(100, &resource_id_result));
  EXPECT_EQ(resource_id_result, resource_id_100_);

  EXPECT_TRUE(resource_id_buffer_.getResourceId(101, &resource_id_result));
  EXPECT_EQ(resource_id_result, resource_id_101_A_);

  EXPECT_TRUE(resource_id_buffer_.getResourceId(200, &resource_id_result));
  EXPECT_EQ(resource_id_result, resource_id_200_);
}

TEST_F(TemporalResourceIdBufferTest, TestDelete) {
  backend::ResourceId resource_id_result;
  EXPECT_TRUE(resource_id_buffer_.getResourceId(101, &resource_id_result));
  EXPECT_EQ(resource_id_result, resource_id_101_A_);

  EXPECT_TRUE(resource_id_buffer_.deleteResourceId(101));

  EXPECT_FALSE(resource_id_buffer_.getResourceId(101, &resource_id_result));
  EXPECT_FALSE(resource_id_buffer_.getResourceId(101, &resource_id_result));

  EXPECT_FALSE(resource_id_buffer_.deleteResourceId(101));
}

TEST_F(TemporalResourceIdBufferTest, TestGetClosestResourceId) {
  backend::StampedResourceId stamped_resource_id_result;

  constexpr int64_t kToleranceHigh = 2;
  constexpr int64_t kToleranceLow = 1;

  // Near, on, and after the first timestamp.
  EXPECT_TRUE(resource_id_buffer_.getClosestResourceId(
      0, kToleranceHigh, &stamped_resource_id_result));
  EXPECT_EQ(stamped_resource_id_result.second, resource_id_2_);

  EXPECT_FALSE(resource_id_buffer_.getClosestResourceId(
      0, kToleranceLow, &stamped_resource_id_result));

  EXPECT_TRUE(resource_id_buffer_.getClosestResourceId(
      2, kToleranceLow, &stamped_resource_id_result));
  EXPECT_EQ(stamped_resource_id_result.second, resource_id_2_);

  EXPECT_TRUE(resource_id_buffer_.getClosestResourceId(
      3, kToleranceLow, &stamped_resource_id_result));
  EXPECT_EQ(stamped_resource_id_result.second, resource_id_2_);

  // Near, on, and after a "normal" timestamp.
  EXPECT_TRUE(resource_id_buffer_.getClosestResourceId(
      8, kToleranceLow, &stamped_resource_id_result));
  EXPECT_EQ(stamped_resource_id_result.second, resource_id_9_);

  EXPECT_TRUE(resource_id_buffer_.getClosestResourceId(
      9, kToleranceLow, &stamped_resource_id_result));
  EXPECT_EQ(stamped_resource_id_result.second, resource_id_9_);

  EXPECT_TRUE(resource_id_buffer_.getClosestResourceId(
      10, kToleranceLow, &stamped_resource_id_result));
  EXPECT_EQ(stamped_resource_id_result.second, resource_id_9_);

  EXPECT_FALSE(resource_id_buffer_.getClosestResourceId(
      11, kToleranceLow, &stamped_resource_id_result));

  // Near, on, and after identical timestamps.
  EXPECT_TRUE(resource_id_buffer_.getClosestResourceId(
      99, kToleranceLow, &stamped_resource_id_result));
  EXPECT_EQ(stamped_resource_id_result.second, resource_id_100_);

  EXPECT_TRUE(resource_id_buffer_.getClosestResourceId(
      100, kToleranceLow, &stamped_resource_id_result));
  EXPECT_EQ(stamped_resource_id_result.second, resource_id_100_);

  EXPECT_TRUE(resource_id_buffer_.getClosestResourceId(
      101, kToleranceLow, &stamped_resource_id_result));
  EXPECT_EQ(stamped_resource_id_result.second, resource_id_101_A_);

  EXPECT_TRUE(resource_id_buffer_.getClosestResourceId(
      102, kToleranceLow, &stamped_resource_id_result));
  EXPECT_EQ(stamped_resource_id_result.second, resource_id_101_A_);

  // Near, on, and after the last timestamp.
  EXPECT_TRUE(resource_id_buffer_.getClosestResourceId(
      199, kToleranceLow, &stamped_resource_id_result));
  EXPECT_EQ(stamped_resource_id_result.second, resource_id_200_);

  EXPECT_TRUE(resource_id_buffer_.getClosestResourceId(
      200, kToleranceLow, &stamped_resource_id_result));
  EXPECT_EQ(stamped_resource_id_result.second, resource_id_200_);

  EXPECT_TRUE(resource_id_buffer_.getClosestResourceId(
      201, kToleranceLow, &stamped_resource_id_result));
  EXPECT_EQ(stamped_resource_id_result.second, resource_id_200_);

  EXPECT_FALSE(resource_id_buffer_.getClosestResourceId(
      202, kToleranceLow, &stamped_resource_id_result));
}

}  // namespace backend

MAPLAB_UNITTEST_ENTRYPOINT
