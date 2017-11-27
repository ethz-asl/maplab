#include <Eigen/Dense>

#include <eigen-checks/gtest.h>
#include <maplab-common/test/testing-entrypoint.h>

#include <vio-common/imu-measurements-buffer.h>
#include <vio-common/vio-types.h>

TEST(ImuMeasurementBuffer, PopFromEmptyBuffer) {
  vio_common::ImuMeasurementBuffer buffer(-1);

  // Pop from empty buffer.
  Eigen::Matrix<int64_t, 1, Eigen::Dynamic> imu_timestamps(1, 2);
  Eigen::Matrix<double, 6, Eigen::Dynamic> imu_measurements(6, 2);
  vio_common::ImuMeasurementBuffer::QueryResult success =
      buffer.getImuDataInterpolatedBorders(
          50, 100, &imu_timestamps, &imu_measurements);

  EXPECT_EQ(
      success,
      vio_common::ImuMeasurementBuffer::QueryResult::kDataNotYetAvailable);
  EXPECT_EQ(0u, imu_timestamps.size());
  EXPECT_EQ(0u, imu_measurements.size());
}

TEST(ImuMeasurementBuffer, LinearInterpolate) {
  vio::ImuData y;
  vio_common::ImuMeasurementBuffer::linearInterpolate(
      10, vio::ImuData::Constant(10.0), 20, vio::ImuData::Constant(50.0), 15,
      &y);
  EXPECT_EQ(y, vio::ImuData::Constant(30.0));
}

TEST(ImuMeasurementBuffer, getImuDataInterpolatedBorders) {
  vio_common::ImuMeasurementBuffer buffer(-1);
  buffer.addMeasurement(10, vio::ImuData::Constant(10.0));
  buffer.addMeasurement(15, vio::ImuData::Constant(15.0));
  buffer.addMeasurement(20, vio::ImuData::Constant(20.0));
  buffer.addMeasurement(25, vio::ImuData::Constant(25.0));
  buffer.addMeasurement(30, vio::ImuData::Constant(30.0));
  buffer.addMeasurement(40, vio::ImuData::Constant(40.0));
  buffer.addMeasurement(50, vio::ImuData::Constant(50.0));

  Eigen::Matrix<int64_t, 1, Eigen::Dynamic> imu_timestamps;
  Eigen::Matrix<double, 6, Eigen::Dynamic> imu_measurements;
  vio_common::ImuMeasurementBuffer::QueryResult result;

  // Test aligned getter (no-interpolation, only border values).
  result = buffer.getImuDataInterpolatedBorders(
      20, 30, &imu_timestamps, &imu_measurements);
  ASSERT_EQ(
      result, vio_common::ImuMeasurementBuffer::QueryResult::kDataAvailable);
  ASSERT_EQ(imu_timestamps.cols(), 3);
  ASSERT_EQ(imu_measurements.cols(), 3);
  EXPECT_EQ(imu_timestamps(0), 20);
  EXPECT_EQ(imu_timestamps(1), 25);
  EXPECT_EQ(imu_timestamps(2), 30);
  EXPECT_EQ(imu_measurements.col(0)(0), 20.0);
  EXPECT_EQ(imu_measurements.col(1)(0), 25.0);
  EXPECT_EQ(imu_measurements.col(2)(0), 30.0);

  // Test aligned getter (no-interpolation).
  result = buffer.getImuDataInterpolatedBorders(
      20, 40, &imu_timestamps, &imu_measurements);
  ASSERT_EQ(
      result, vio_common::ImuMeasurementBuffer::QueryResult::kDataAvailable);
  ASSERT_EQ(imu_timestamps.cols(), 4);
  ASSERT_EQ(imu_measurements.cols(), 4);
  EXPECT_EQ(imu_timestamps(0), 20);
  EXPECT_EQ(imu_timestamps(1), 25);
  EXPECT_EQ(imu_timestamps(2), 30);
  EXPECT_EQ(imu_timestamps(3), 40);
  EXPECT_EQ(imu_measurements.col(0)(0), 20.0);
  EXPECT_EQ(imu_measurements.col(1)(0), 25.0);
  EXPECT_EQ(imu_measurements.col(2)(0), 30.0);
  EXPECT_EQ(imu_measurements.col(3)(0), 40.0);

  // Test unaligned getter (lower/upper-interpolation).
  result = buffer.getImuDataInterpolatedBorders(
      19, 21, &imu_timestamps, &imu_measurements);
  ASSERT_EQ(
      result, vio_common::ImuMeasurementBuffer::QueryResult::kDataAvailable);
  ASSERT_EQ(imu_timestamps.cols(), 3);
  ASSERT_EQ(imu_measurements.cols(), 3);
  EXPECT_EQ(imu_timestamps(0), 19);
  EXPECT_EQ(imu_timestamps(1), 20);
  EXPECT_EQ(imu_timestamps(2), 21);
  EXPECT_EQ(imu_measurements.col(0)(0), 19.0);
  EXPECT_EQ(imu_measurements.col(1)(0), 20.0);
  EXPECT_EQ(imu_measurements.col(2)(0), 21.0);

  // Fail: query out of upper bound.
  result = buffer.getImuDataInterpolatedBorders(
      40, 51, &imu_timestamps, &imu_measurements);
  ASSERT_EQ(
      result,
      vio_common::ImuMeasurementBuffer::QueryResult::kDataNotYetAvailable);
  result = buffer.getImuDataInterpolatedBorders(
      60, 61, &imu_timestamps, &imu_measurements);
  ASSERT_EQ(
      result,
      vio_common::ImuMeasurementBuffer::QueryResult::kDataNotYetAvailable);

  // Fail: query out of lower bound.
  result = buffer.getImuDataInterpolatedBorders(
      -1, 20, &imu_timestamps, &imu_measurements);
  ASSERT_EQ(
      result,
      vio_common::ImuMeasurementBuffer::QueryResult::kDataNeverAvailable);
  result = buffer.getImuDataInterpolatedBorders(
      -20, -10, &imu_timestamps, &imu_measurements);
  ASSERT_EQ(
      result,
      vio_common::ImuMeasurementBuffer::QueryResult::kDataNeverAvailable);

  // Query between two values: return the border values.
  result = buffer.getImuDataInterpolatedBorders(
      21, 29, &imu_timestamps, &imu_measurements);
  ASSERT_EQ(
      result, vio_common::ImuMeasurementBuffer::QueryResult::kDataAvailable);
  ASSERT_EQ(imu_timestamps.cols(), 3);
  ASSERT_EQ(imu_measurements.cols(), 3);
  EXPECT_EQ(imu_timestamps(0), 21);
  EXPECT_EQ(imu_timestamps(1), 25);
  EXPECT_EQ(imu_timestamps(2), 29);
  EXPECT_EQ(imu_measurements.col(0)(0), 21.0);
  EXPECT_EQ(imu_measurements.col(1)(0), 25.0);
  EXPECT_EQ(imu_measurements.col(2)(0), 29.0);
}

TEST(ImuMeasurementBuffer, DeathOnAddDataNotIncreasingTimestamp) {
  vio_common::ImuMeasurementBuffer buffer(-1);

  Eigen::Matrix<double, 6, 1> imu_measurement;
  imu_measurement.setRandom();

  buffer.addMeasurement(0u, imu_measurement);
  buffer.addMeasurement(10u, imu_measurement);
  EXPECT_DEATH(buffer.addMeasurement(9u, imu_measurement), "^");
}

TEST(ImuMeasurementBuffer, TestAddMeasurements) {
  const size_t kNumMeasurements = 10;
  vio_common::ImuMeasurementBuffer buffer(-1);

  // Create IMU measurements and fill buffer.
  Eigen::Matrix<int64_t, 1, Eigen::Dynamic> imu_timestamps_groundtruth(
      1, kNumMeasurements);
  Eigen::Matrix<double, 6, Eigen::Dynamic> imu_measurements_groundtruth(
      6, kNumMeasurements);

  for (size_t idx = 0; idx < kNumMeasurements; ++idx) {
    int64_t timestamp = static_cast<int64_t>(idx * 10);
    Eigen::Matrix<double, 6, 1> imu_measurement;
    imu_measurement.setConstant(idx);
    imu_timestamps_groundtruth(idx) = timestamp;
    imu_measurements_groundtruth.col(idx) = imu_measurement;
  }
  buffer.addMeasurements(
      imu_timestamps_groundtruth, imu_measurements_groundtruth);
}

MAPLAB_UNITTEST_ENTRYPOINT
