#include <eigen-checks/gtest.h>
#include <gtest/gtest.h>

#include <Eigen/Core>
#include <maplab-common/test/testing-entrypoint.h>

#define ENABLE_VIZDATA_COLLECTOR 1
#include "visualization/viz-data-collector.h"

namespace visualization {
class VizCollectorTest : public ::testing::Test {
 protected:
  virtual void SetUp() {}
  virtual void TearDown() {
    // Reset the singleton.
    VizDataCollector::Instance().reset();
  }
};

const aslam::NFramesId kSlotId = aslam::NFramesId::Random();
const std::string kChannelName("test-channel");
const std::string kTestData("test-data");

TEST_F(VizCollectorTest, PushAndReadData) {
  VizDataCollector::Instance().pushData(kSlotId, kChannelName, kTestData);

  const std::string* data;
  EXPECT_TRUE(
      VizDataCollector::Instance().getDataSafe(kSlotId, kChannelName, &data));
  ASSERT_TRUE(data != nullptr);
  EXPECT_EQ(*data, kTestData);
}

TEST_F(VizCollectorTest, HasSlotAndRemoveSlot) {
  VizDataCollector::Instance().pushData(kSlotId, kChannelName, kTestData);
  EXPECT_TRUE(VizDataCollector::Instance().hasSlot(kSlotId));

  VizDataCollector::Instance().removeSlotIfAvailable(kSlotId);
  EXPECT_FALSE(VizDataCollector::Instance().hasSlot(kSlotId));
}

TEST_F(VizCollectorTest, HasChannelAndRemoveSlot) {
  VizDataCollector::Instance().pushData(kSlotId, kChannelName, kTestData);
  EXPECT_TRUE(VizDataCollector::Instance().hasChannel(kSlotId, kChannelName));

  VizDataCollector::Instance().removeSlotIfAvailable(kSlotId);
  EXPECT_FALSE(VizDataCollector::Instance().hasChannel(kSlotId, kChannelName));
}

TEST_F(VizCollectorTest, PrintChannel) {
  VizDataCollector::Instance().pushData(kSlotId, kChannelName, kTestData);
  EXPECT_EQ(
      VizDataCollector::Instance().printData<std::string>(
          kSlotId, kChannelName),
      kTestData);

  const aslam::NFramesId kUnavailableSlotId = aslam::NFramesId::Random();
  const std::string kUnavailableChannel("channel-na");
  const std::string kUnavailabeMessage("Channel not available.");
  EXPECT_EQ(
      VizDataCollector::Instance().printData<std::string>(
          kUnavailableSlotId, kChannelName),
      kUnavailabeMessage);
  EXPECT_EQ(
      VizDataCollector::Instance().printData<std::string>(
          kSlotId, kUnavailableChannel),
      kUnavailabeMessage);
  EXPECT_EQ(
      VizDataCollector::Instance().printData<std::string>(
          kUnavailableSlotId, kUnavailableChannel),
      kUnavailabeMessage);

  LOG(INFO) << VizDataCollector::PrintChannel<std::string>(
      kSlotId, kChannelName);
}

}  // namespace visualization
MAPLAB_UNITTEST_ENTRYPOINT
