#include <thread>

#include <Eigen/Dense>
#include <eigen-checks/gtest.h>

#include "maplab-common/interpolation-helpers.h"
#include "maplab-common/test/testing-entrypoint.h"
#include "maplab-common/threadsafe-temporal-buffer.h"

namespace common {

typedef ThreadsafeTemporalBuffer<double> BufferType;

TEST(AsyncTemporalBufferTest, PopFromEmptyBuffer) {
  constexpr int64_t kInfiniteBufferLength = -1;
  BufferType buffer(kInfiniteBufferLength);

  // Pop from empty buffer.
  std::vector<double> data;
  BufferType::QueryResult success =
      buffer.getInterpolatedBorders(50, 100, &data);

  EXPECT_EQ(success, BufferType::QueryResult::kDataNotYetAvailable);
  EXPECT_TRUE(data.empty());
}

TEST(AsyncTemporalBufferTest, getInterpolatedBorders) {
  constexpr int64_t kInfiniteBufferLength = -1;
  BufferType buffer(kInfiniteBufferLength);

  buffer.pushBack(10, 10.0);
  buffer.pushBack(15, 15.0);
  buffer.pushBack(20, 20.0);
  buffer.pushBack(25, 25.0);
  buffer.pushBack(30, 30.0);
  buffer.pushBack(40, 40.0);
  buffer.pushBack(50, 50.0);
  std::vector<double> data;
  BufferType::QueryResult result;

  // Test aligned getter (no-interpolation, only border values).
  result = buffer.getInterpolatedBorders(20, 30, &data);
  ASSERT_EQ(result, BufferType::QueryResult::kDataAvailable);
  ASSERT_EQ(data.size(), 3u);
  EXPECT_EQ(data[0], 20.0);
  EXPECT_EQ(data[1], 25.0);
  EXPECT_EQ(data[2], 30.0);
  // Test aligned getter (no-interpolation).
  result = buffer.getInterpolatedBorders(20, 40, &data);
  ASSERT_EQ(result, BufferType::QueryResult::kDataAvailable);
  ASSERT_EQ(data.size(), 4u);
  EXPECT_EQ(data[0], 20.0);
  EXPECT_EQ(data[1], 25.0);
  EXPECT_EQ(data[2], 30.0);
  EXPECT_EQ(data[3], 40.0);

  // Test unaligned getter (lower/upper-interpolation).
  result = buffer.getInterpolatedBorders(19, 21, &data);
  ASSERT_EQ(result, BufferType::QueryResult::kDataAvailable);
  ASSERT_EQ(data.size(), 3u);
  EXPECT_EQ(data[0], 19.0);
  EXPECT_EQ(data[1], 20.0);
  EXPECT_EQ(data[2], 21.0);

  // Fail: query out of upper bound.
  result = buffer.getInterpolatedBorders(40, 51, &data);
  ASSERT_EQ(result, BufferType::QueryResult::kDataNotYetAvailable);
  result = buffer.getInterpolatedBorders(60, 61, &data);
  ASSERT_EQ(result, BufferType::QueryResult::kDataNotYetAvailable);

  // Fail: query out of lower bound.
  result = buffer.getInterpolatedBorders(-1, 20, &data);
  ASSERT_EQ(result, BufferType::QueryResult::kDataNeverAvailable);
  result = buffer.getInterpolatedBorders(-20, -10, &data);
  ASSERT_EQ(result, BufferType::QueryResult::kDataNeverAvailable);

  // Query between two values: return the border values.
  result = buffer.getInterpolatedBorders(21, 29, &data);
  ASSERT_EQ(result, BufferType::QueryResult::kDataAvailable);
  ASSERT_EQ(data.size(), 3);
  EXPECT_EQ(data[0], 21.0);
  EXPECT_EQ(data[1], 25.0);
  EXPECT_EQ(data[2], 29.0);
}

TEST(AsyncTemporalBufferTest, DeathOnAddDataNotIncreasingTimestamp) {
  constexpr int64_t kInfiniteBufferLength = -1;
  BufferType buffer(kInfiniteBufferLength);

  constexpr double kRandomValue1 = 139084.3;
  constexpr double kRandomValue2 = -389.325;
  constexpr double kRandomValue3 = 10.0;

  buffer.pushBack(0, kRandomValue1);
  buffer.pushBack(10, kRandomValue2);
  EXPECT_DEATH(buffer.pushBack(9, kRandomValue3), "^");
}

constexpr size_t kNumElements = 99999u;

void fillBuffer(BufferType* buffer) {
  CHECK_NOTNULL(buffer);
  for (size_t i = 0u; i < kNumElements; ++i) {
    buffer->pushBack(static_cast<int64_t>(i), static_cast<double>(i));
  }
}

void queryBufferGetInterpolatedBordersBlocking(const BufferType* buffer) {
  constexpr int64_t kWaitTimeoutNanoseconds = static_cast<int64_t>(1e9);  // 1s.
  for (size_t i = 0u; i < kNumElements - 2u; ++i) {
    std::vector<double> values;
    buffer->getInterpolatedBordersBlocking(
        static_cast<int64_t>(i), static_cast<int64_t>(i + 2),
        kWaitTimeoutNanoseconds, &values);
    ASSERT_EQ(values.size(), 3u);
    EXPECT_EQ(values[0], static_cast<double>(i));
    EXPECT_EQ(values[1], static_cast<double>(i + 1));
    EXPECT_EQ(values[2], static_cast<double>(i + 2));
  }
}

void queryBufferGetValuesFromExcludingToIncludingBlocking(
    const BufferType* buffer) {
  constexpr int64_t kWaitTimeoutNanoseconds = static_cast<int64_t>(1e9);  // 1s.
  for (size_t i = 0u; i < kNumElements - 2u; ++i) {
    std::vector<double> values;
    buffer->getValuesFromExcludingToIncludingBlocking(
        static_cast<int64_t>(i), static_cast<int64_t>(i + 2),
        kWaitTimeoutNanoseconds, &values);
    ASSERT_EQ(values.size(), 2u);
    EXPECT_EQ(values[0], static_cast<double>(i + 1));
    EXPECT_EQ(values[1], static_cast<double>(i + 2));
  }
}

TEST(AsyncTemporalBufferTest, ConcurrentAccess) {
  constexpr int64_t kInfiniteBufferLength = -1;
  BufferType buffer(kInfiniteBufferLength);

  std::thread consumer_thread1(
      queryBufferGetInterpolatedBordersBlocking, &buffer);
  std::thread consumer_thread2(
      queryBufferGetValuesFromExcludingToIncludingBlocking, &buffer);

  std::this_thread::sleep_for(std::chrono::milliseconds(500));
  std::thread producer_thread(fillBuffer, &buffer);

  producer_thread.join();
  consumer_thread1.join();
  consumer_thread2.join();
}

}  // namespace common
MAPLAB_UNITTEST_ENTRYPOINT
