#include <glog/logging.h>
#include <gtest/gtest.h>

#include <aslam/common/entrypoint.h>
#include <aslam/common/time.h>

TEST(TestTime, TestCompileTimeConverters) {
  EXPECT_EQ(10e9, aslam::time::seconds(10));
  EXPECT_EQ(10e6, aslam::time::milliseconds(10));
  EXPECT_EQ(10e3, aslam::time::microseconds(10));
  EXPECT_EQ(10, aslam::time::nanoseconds(10));
}

TEST(TestTime, TestToTimestampConverters) {
  const double kTolerance = 1e-16;

  EXPECT_NEAR(1.0, aslam::time::to_seconds(1e9), kTolerance);
  EXPECT_NEAR(0.5, aslam::time::to_seconds(0.5e9), kTolerance);

  EXPECT_NEAR(1.0, aslam::time::to_milliseconds(1e6), kTolerance);
  EXPECT_NEAR(0.5, aslam::time::to_milliseconds(0.5e6), kTolerance);

  EXPECT_NEAR(1.0, aslam::time::to_microseconds(1e3), kTolerance);
  EXPECT_NEAR(0.5, aslam::time::to_microseconds(0.5e3), kTolerance);
}

TEST(TestTime, TestFromTimestampConverters) {
  EXPECT_EQ(1e9, aslam::time::from_seconds(1.0));
  EXPECT_EQ(5e8, aslam::time::from_seconds(0.5));

  EXPECT_EQ(1e6, aslam::time::from_milliseconds(1.0));
  EXPECT_EQ(5e5, aslam::time::from_milliseconds(0.5));

  EXPECT_EQ(1e3, aslam::time::from_microseconds(1.0));
  EXPECT_EQ(5e2, aslam::time::from_microseconds(0.5));
}

TEST(TestTime, TestCompareCompileTimeAndRuntimeConverters) {
  EXPECT_EQ(aslam::time::seconds(10), aslam::time::from_seconds(10.0));
  EXPECT_EQ(aslam::time::milliseconds(10), aslam::time::from_milliseconds(10.0));
  EXPECT_EQ(aslam::time::microseconds(10), aslam::time::from_microseconds(10.0));
}

TEST(TestTime, TestInvalidTimeMagicTimestamp) {
  int64_t invalid_timestamp = aslam::time::getInvalidTime();
  EXPECT_FALSE(aslam::time::isValidTime(invalid_timestamp));
}

TEST(TestTime, TestGetCurrentTime) {
  int64_t timestamp_now = aslam::time::nanoSecondsSinceEpoch();
  EXPECT_TRUE(aslam::time::isValidTime(timestamp_now));
}

TEST(TestTime, TestTimeNanosecondsToString) {
  constexpr int64_t time_ns_1 = 1234567890;
  EXPECT_EQ(aslam::time::timeNanosecondsToString(time_ns_1), "1s 234567890ns");
  constexpr int64_t time_ns_2 = 123456789;
  EXPECT_EQ(aslam::time::timeNanosecondsToString(time_ns_2), "123456789ns");
}

ASLAM_UNITTEST_ENTRYPOINT
