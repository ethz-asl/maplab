#include <thread>

#include <aslam/common/time.h>
#include <maplab-common/temporal-buffer.h>

#include "maplab-common/test/testing-entrypoint.h"

namespace common {

struct TestData {
  explicit TestData(int64_t time) : timestamp(time) {}
  TestData() = default;

  int64_t timestamp;
};

class TemporalBufferFixture : public ::testing::Test {
 public:
  TemporalBufferFixture() : buffer_(kBufferLengthNs) {}

 protected:
  virtual void SetUp() {}
  virtual void TearDown() {}

  void addValue(const TestData& data) {
    buffer_.addValue(data.timestamp, data);
  }

  static constexpr int64_t kBufferLengthNs = 100;
  TemporalBuffer<TestData> buffer_;
};

TEST_F(TemporalBufferFixture, SizeEmptyClearWork) {
  EXPECT_TRUE(buffer_.empty());
  EXPECT_EQ(buffer_.size(), 0u);

  addValue(TestData(10));
  addValue(TestData(20));
  EXPECT_FALSE(buffer_.empty());
  EXPECT_EQ(buffer_.size(), 2u);

  buffer_.clear();
  EXPECT_TRUE(buffer_.empty());
  EXPECT_EQ(buffer_.size(), 0u);
}

TEST_F(TemporalBufferFixture, GetValueAtTimeWorks) {
  addValue(TestData(30));
  addValue(TestData(10));
  addValue(TestData(20));
  addValue(TestData(40));

  TestData retrieved_item;
  EXPECT_TRUE(buffer_.getValueAtTime(10, &retrieved_item));
  EXPECT_EQ(retrieved_item.timestamp, 10);

  EXPECT_TRUE(buffer_.getValueAtTime(20, &retrieved_item));
  EXPECT_EQ(retrieved_item.timestamp, 20);

  EXPECT_FALSE(buffer_.getValueAtTime(15, &retrieved_item));

  EXPECT_TRUE(buffer_.getValueAtTime(30, &retrieved_item));
  EXPECT_EQ(retrieved_item.timestamp, 30);
}

TEST_F(TemporalBufferFixture, GetNearestValueToTimeWorks) {
  addValue(TestData(30));
  addValue(TestData(10));
  addValue(TestData(20));

  TestData retrieved_item;
  EXPECT_TRUE(buffer_.getNearestValueToTime(10, &retrieved_item));
  EXPECT_EQ(retrieved_item.timestamp, 10);

  EXPECT_TRUE(buffer_.getNearestValueToTime(0, &retrieved_item));
  EXPECT_EQ(retrieved_item.timestamp, 10);

  EXPECT_TRUE(buffer_.getNearestValueToTime(16, &retrieved_item));
  EXPECT_EQ(retrieved_item.timestamp, 20);

  EXPECT_TRUE(buffer_.getNearestValueToTime(26, &retrieved_item));
  EXPECT_EQ(retrieved_item.timestamp, 30);

  EXPECT_TRUE(buffer_.getNearestValueToTime(32, &retrieved_item));
  EXPECT_EQ(retrieved_item.timestamp, 30);

  EXPECT_TRUE(buffer_.getNearestValueToTime(1232, &retrieved_item));
  EXPECT_EQ(retrieved_item.timestamp, 30);
}

TEST_F(TemporalBufferFixture, GetNearestValueToTimeMaxDeltaWorks) {
  addValue(TestData(40));
  addValue(TestData(10));
  addValue(TestData(20));

  const int kMaxDelta = 5;

  TestData retrieved_item;
  EXPECT_TRUE(buffer_.getNearestValueToTime(10, kMaxDelta, &retrieved_item));
  EXPECT_EQ(retrieved_item.timestamp, 10);

  EXPECT_FALSE(buffer_.getNearestValueToTime(0, kMaxDelta, &retrieved_item));

  EXPECT_TRUE(buffer_.getNearestValueToTime(9, kMaxDelta, &retrieved_item));
  EXPECT_EQ(retrieved_item.timestamp, 10);

  EXPECT_TRUE(buffer_.getNearestValueToTime(16, kMaxDelta, &retrieved_item));
  EXPECT_EQ(retrieved_item.timestamp, 20);

  EXPECT_TRUE(buffer_.getNearestValueToTime(36, kMaxDelta, &retrieved_item));
  EXPECT_EQ(retrieved_item.timestamp, 40);

  EXPECT_TRUE(buffer_.getNearestValueToTime(42, kMaxDelta, &retrieved_item));
  EXPECT_EQ(retrieved_item.timestamp, 40);

  EXPECT_FALSE(buffer_.getNearestValueToTime(46, kMaxDelta, &retrieved_item));

  buffer_.clear();
  addValue(TestData(10));
  addValue(TestData(20));

  EXPECT_TRUE(buffer_.getNearestValueToTime(9, kMaxDelta, &retrieved_item));
  EXPECT_EQ(retrieved_item.timestamp, 10);

  EXPECT_TRUE(buffer_.getNearestValueToTime(12, kMaxDelta, &retrieved_item));
  EXPECT_EQ(retrieved_item.timestamp, 10);

  EXPECT_TRUE(buffer_.getNearestValueToTime(16, kMaxDelta, &retrieved_item));
  EXPECT_EQ(retrieved_item.timestamp, 20);

  EXPECT_TRUE(buffer_.getNearestValueToTime(22, kMaxDelta, &retrieved_item));
  EXPECT_EQ(retrieved_item.timestamp, 20);

  buffer_.clear();
  addValue(TestData(10));

  EXPECT_FALSE(buffer_.getNearestValueToTime(1, kMaxDelta, &retrieved_item));

  EXPECT_TRUE(buffer_.getNearestValueToTime(6, kMaxDelta, &retrieved_item));
  EXPECT_EQ(retrieved_item.timestamp, 10);

  EXPECT_TRUE(buffer_.getNearestValueToTime(14, kMaxDelta, &retrieved_item));
  EXPECT_EQ(retrieved_item.timestamp, 10);

  EXPECT_FALSE(buffer_.getNearestValueToTime(16, kMaxDelta, &retrieved_item));
}

TEST_F(TemporalBufferFixture, GetValueAtOrBeforeTimeWorks) {
  addValue(TestData(30));
  addValue(TestData(10));
  addValue(TestData(20));
  addValue(TestData(40));

  TestData retrieved_item;
  int64_t timestamp;

  EXPECT_TRUE(buffer_.getValueAtOrBeforeTime(40, &timestamp, &retrieved_item));
  EXPECT_EQ(retrieved_item.timestamp, 40);
  EXPECT_EQ(timestamp, 40);

  EXPECT_TRUE(buffer_.getValueAtOrBeforeTime(50, &timestamp, &retrieved_item));
  EXPECT_EQ(retrieved_item.timestamp, 40);
  EXPECT_EQ(timestamp, 40);

  EXPECT_TRUE(buffer_.getValueAtOrBeforeTime(15, &timestamp, &retrieved_item));
  EXPECT_EQ(retrieved_item.timestamp, 10);
  EXPECT_EQ(timestamp, 10);

  EXPECT_TRUE(buffer_.getValueAtOrBeforeTime(10, &timestamp, &retrieved_item));
  EXPECT_EQ(retrieved_item.timestamp, 10);
  EXPECT_EQ(timestamp, 10);

  EXPECT_FALSE(buffer_.getValueAtOrBeforeTime(5, &timestamp, &retrieved_item));
}

TEST_F(TemporalBufferFixture, GetValueAtOrAfterTimeWorks) {
  addValue(TestData(30));
  addValue(TestData(10));
  addValue(TestData(20));
  addValue(TestData(40));

  TestData retrieved_item;
  int64_t timestamp;

  EXPECT_TRUE(buffer_.getValueAtOrAfterTime(10, &timestamp, &retrieved_item));
  EXPECT_EQ(retrieved_item.timestamp, 10);
  EXPECT_EQ(timestamp, 10);

  EXPECT_TRUE(buffer_.getValueAtOrAfterTime(5, &timestamp, &retrieved_item));
  EXPECT_EQ(retrieved_item.timestamp, 10);
  EXPECT_EQ(timestamp, 10);

  EXPECT_TRUE(buffer_.getValueAtOrAfterTime(35, &timestamp, &retrieved_item));
  EXPECT_EQ(retrieved_item.timestamp, 40);
  EXPECT_EQ(timestamp, 40);

  EXPECT_TRUE(buffer_.getValueAtOrAfterTime(40, &timestamp, &retrieved_item));
  EXPECT_EQ(retrieved_item.timestamp, 40);
  EXPECT_EQ(timestamp, 40);

  EXPECT_FALSE(buffer_.getValueAtOrAfterTime(45, &timestamp, &retrieved_item));
}

TEST_F(TemporalBufferFixture, GetOldestNewestValueWork) {
  TestData retrieved_item;
  EXPECT_FALSE(buffer_.getOldestValue(&retrieved_item));
  EXPECT_FALSE(buffer_.getNewestValue(&retrieved_item));

  addValue(TestData(30));
  addValue(TestData(10));
  addValue(TestData(20));
  addValue(TestData(40));

  EXPECT_TRUE(buffer_.getOldestValue(&retrieved_item));
  EXPECT_EQ(retrieved_item.timestamp, 10);

  EXPECT_TRUE(buffer_.getNewestValue(&retrieved_item));
  EXPECT_EQ(retrieved_item.timestamp, 40);
}

TEST_F(TemporalBufferFixture, GetValuesBetweenTimesWorks) {
  addValue(TestData(10));
  addValue(TestData(20));
  addValue(TestData(30));
  addValue(TestData(40));
  addValue(TestData(50));

  // Test aligned borders.
  std::vector<TestData> values;
  buffer_.getValuesBetweenTimes(10, 50, &values);
  ASSERT_EQ(values.size(), 3u);
  EXPECT_EQ(values[0].timestamp, 20);
  EXPECT_EQ(values[1].timestamp, 30);
  EXPECT_EQ(values[2].timestamp, 40);

  // Test unaligned borders.
  buffer_.getValuesBetweenTimes(15, 45, &values);
  ASSERT_EQ(values.size(), 3u);
  EXPECT_EQ(values[0].timestamp, 20);
  EXPECT_EQ(values[1].timestamp, 30);
  EXPECT_EQ(values[2].timestamp, 40);

  // Test lower than first item.
  buffer_.getValuesBetweenTimes(5, 45, &values);
  ASSERT_EQ(values.size(), 4u);
  EXPECT_EQ(values[0].timestamp, 10);
  EXPECT_EQ(values[1].timestamp, 20);
  EXPECT_EQ(values[2].timestamp, 30);
  EXPECT_EQ(values[3].timestamp, 40);

  // Test higher than last item.
  buffer_.getValuesBetweenTimes(30, 55, &values);
  ASSERT_EQ(values.size(), 2u);
  EXPECT_EQ(values[0].timestamp, 40);
  EXPECT_EQ(values[1].timestamp, 50);

  // Return empty list if there none in the buffer.
  buffer_.clear();
  buffer_.getValuesBetweenTimes(10, 50, &values);
  EXPECT_TRUE(values.empty());

  // Expect check-fail when the lower query timestamp is higher than the
  // higher query timestamp.
  EXPECT_DEATH(buffer_.getValuesBetweenTimes(40, 30, &values), "");
  EXPECT_TRUE(values.empty());
}

TEST_F(TemporalBufferFixture, MaintaingBufferLengthWorks) {
  addValue(TestData(0));
  addValue(TestData(50));
  addValue(TestData(100));
  EXPECT_EQ(buffer_.size(), 3u);

  addValue(TestData(150));
  EXPECT_EQ(buffer_.size(), 3u);

  TestData retrieved_item;
  EXPECT_TRUE(buffer_.getOldestValue(&retrieved_item));
  EXPECT_EQ(retrieved_item.timestamp, 50);

  EXPECT_TRUE(buffer_.getNewestValue(&retrieved_item));
  EXPECT_EQ(retrieved_item.timestamp, 150);
}

TEST_F(TemporalBufferFixture, RemovingItemsByThreshold) {
  addValue(TestData(70));
  addValue(TestData(99));
  addValue(TestData(100));
  addValue(TestData(101));
  addValue(TestData(110));
  EXPECT_EQ(buffer_.size(), 5u);

  EXPECT_EQ(buffer_.removeItemsBefore(100), 2u);
  EXPECT_EQ(buffer_.size(), 3u);

  TestData retrieved_item;
  EXPECT_TRUE(buffer_.getOldestValue(&retrieved_item));
  EXPECT_EQ(retrieved_item.timestamp, 100);

  EXPECT_TRUE(buffer_.getNewestValue(&retrieved_item));
  EXPECT_EQ(retrieved_item.timestamp, 110);

  EXPECT_EQ(buffer_.removeItemsBefore(120), 3u);
  EXPECT_EQ(buffer_.size(), 0u);
}

TEST_F(TemporalBufferFixture, RemovingAndRetrieveItemsByThreshold) {
  addValue(TestData(70));
  addValue(TestData(99));
  addValue(TestData(100));
  addValue(TestData(101));
  addValue(TestData(110));
  EXPECT_EQ(buffer_.size(), 5u);

  std::vector<TestData> removed_values;
  EXPECT_EQ(buffer_.extractItemsBeforeIncluding(100, &removed_values), 3u);
  EXPECT_EQ(buffer_.size(), 2u);
  ASSERT_EQ(removed_values.size(), 3u);
  EXPECT_EQ(removed_values[0].timestamp, 70);
  EXPECT_EQ(removed_values[1].timestamp, 99);
  EXPECT_EQ(removed_values[2].timestamp, 100);

  // Try to remove values older the buffer range.
  EXPECT_EQ(buffer_.extractItemsBeforeIncluding(10, &removed_values), 0u);
  EXPECT_EQ(buffer_.size(), 2u);
  ASSERT_TRUE(removed_values.empty());

  // Try to remove based on threshold newer than the buffer range.
  EXPECT_EQ(buffer_.extractItemsBeforeIncluding(200, &removed_values), 2u);
  EXPECT_EQ(buffer_.size(), 0u);
  ASSERT_EQ(removed_values.size(), 2u);
  EXPECT_EQ(removed_values[0].timestamp, 101);
  EXPECT_EQ(removed_values[1].timestamp, 110);

  // Try to remove values in an empty buffer.
  EXPECT_EQ(buffer_.extractItemsBeforeIncluding(110, &removed_values), 0u);
  EXPECT_EQ(buffer_.size(), 0u);
  ASSERT_TRUE(removed_values.empty());
}

TEST_F(TemporalBufferFixture, GetValuesFromIncludingToIncluding) {
  addValue(TestData(10));
  addValue(TestData(20));
  addValue(TestData(30));
  addValue(TestData(40));
  addValue(TestData(50));

  // Test aligned borders.
  std::vector<TestData> values;
  buffer_.getValuesFromIncludingToIncluding(20, 40, &values);
  ASSERT_EQ(values.size(), 3u);
  EXPECT_EQ(values[0].timestamp, 20);
  EXPECT_EQ(values[1].timestamp, 30);
  EXPECT_EQ(values[2].timestamp, 40);

  // Test unaligned borders.
  buffer_.getValuesFromIncludingToIncluding(15, 45, &values);
  ASSERT_EQ(values.size(), 3u);
  EXPECT_EQ(values[0].timestamp, 20);
  EXPECT_EQ(values[1].timestamp, 30);
  EXPECT_EQ(values[2].timestamp, 40);

  // Test lower than first item.
  buffer_.getValuesFromIncludingToIncluding(5, 45, &values);
  ASSERT_EQ(values.size(), 4u);
  EXPECT_EQ(values[0].timestamp, 10);
  EXPECT_EQ(values[1].timestamp, 20);
  EXPECT_EQ(values[2].timestamp, 30);
  EXPECT_EQ(values[3].timestamp, 40);

  // Test higher than last item.
  buffer_.getValuesFromIncludingToIncluding(30, 55, &values);
  ASSERT_EQ(values.size(), 3u);
  EXPECT_EQ(values[0].timestamp, 30);
  EXPECT_EQ(values[1].timestamp, 40);
  EXPECT_EQ(values[2].timestamp, 50);

  // Return empty list if there nothing in that range in the buffer.
  buffer_.getValuesFromIncludingToIncluding(21, 29, &values);
  EXPECT_TRUE(values.empty());

  // Return empty list if there none in the buffer.
  buffer_.clear();
  buffer_.getValuesFromIncludingToIncluding(10, 50, &values);
  EXPECT_TRUE(values.empty());

  // Expect check-fail when the lower query timestamp is higher than the
  // higher query timestamp.
  EXPECT_DEATH(buffer_.getValuesFromIncludingToIncluding(40, 30, &values), "");
  EXPECT_TRUE(values.empty());
}

}  // namespace common
MAPLAB_UNITTEST_ENTRYPOINT
