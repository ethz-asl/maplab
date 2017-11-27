#include <random>

#include <gtest/gtest.h>
#include <maplab-common/test/testing-entrypoint.h>
#include <maplab-common/test/testing-predicates.h>

#include "sensors/gps-utm.h"
#include "sensors/gps-wgs.h"
#include "sensors/relative-6dof-pose.h"

namespace vi_map {

class MeasurementsTest : public ::testing::Test {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 protected:
  virtual void SetUp() {
    constexpr size_t kSeed = 42u;
    random_engine_ = std::default_random_engine(kSeed);

    constexpr int64_t kMinTimestampNanoseconds = 0;
    constexpr int64_t kMaxTimestampNanoseconds = static_cast<int64_t>(1e14);

    timestamp_distribution_ = std::uniform_int_distribution<int64_t>(
        kMinTimestampNanoseconds, kMaxTimestampNanoseconds);
  }

  int64_t getRandomTimestampNanoseconds() {
    return timestamp_distribution_(random_engine_);
  }

  double getRandomUniformDouble(const double min, const double max) {
    std::uniform_real_distribution<double> distribution(min, max);
    return distribution(random_engine_);
  }

 private:
  std::default_random_engine random_engine_;
  std::uniform_int_distribution<int64_t> timestamp_distribution_;
};

TEST_F(MeasurementsTest, TestAccessorsGpsUtm) {
  // Testing construction with invalid id.
  SensorId sensor_id;
  CHECK(!sensor_id.isValid());
  EXPECT_DEATH(GpsUtmMeasurement gps_utm_measurement(sensor_id), "");

  int64_t timestamp_nanoseconds = 0;
  EXPECT_DEATH(
      GpsUtmMeasurement(
          sensor_id, timestamp_nanoseconds, aslam::Transformation(),
          UtmZone(32u, 'N')),
      "");
  timestamp_nanoseconds = -1;

  // Testing sensor id getter.
  common::generateId(&sensor_id);
  GpsUtmMeasurement gps_utm_measurement(sensor_id);
  EXPECT_EQ(gps_utm_measurement.getSensorId(), sensor_id);

  // Testing construction with invalid timestamp.
  EXPECT_DEATH(
      GpsUtmMeasurement(
          sensor_id, timestamp_nanoseconds, aslam::Transformation(),
          UtmZone::createInvalid()),
      "");

  // Testing timestamp and T_UTM_S getters.
  timestamp_nanoseconds = getRandomTimestampNanoseconds();
  aslam::Transformation T_UTM_S;
  T_UTM_S.setRandom();
  gps_utm_measurement = GpsUtmMeasurement(
      sensor_id, timestamp_nanoseconds, T_UTM_S, UtmZone::createInvalid());
  EXPECT_EQ(
      gps_utm_measurement.getTimestampNanoseconds(), timestamp_nanoseconds);
  EXPECT_EQ(gps_utm_measurement.get_T_UTM_S(), T_UTM_S);
}

TEST_F(MeasurementsTest, TestAccessorsGpsWgs) {
  // Testing construction with invalid id.
  SensorId sensor_id;
  EXPECT_DEATH(GpsWgsMeasurement gps_wgs_measurement(sensor_id), "");

  common::generateId(&sensor_id);
  GpsWgsMeasurement gps_wgs_measurement(sensor_id);
  EXPECT_EQ(gps_wgs_measurement.getSensorId(), sensor_id);

  // Testing construction with invalid GPS values.
  double latidude_deg = 91.0;
  double longitude_deg = -181.0;
  double altitude_meters = -1e10;
  int64_t timestamp_nanoseconds = -1;
  EXPECT_DEATH(
      GpsWgsMeasurement(
          sensor_id, timestamp_nanoseconds, latidude_deg, longitude_deg,
          altitude_meters),
      "");
  timestamp_nanoseconds = 0;
  EXPECT_DEATH(
      GpsWgsMeasurement(
          sensor_id, timestamp_nanoseconds, latidude_deg, longitude_deg,
          altitude_meters),
      "");
  latidude_deg = getRandomUniformDouble(
      GpsWgsMeasurement::kMinLatitudeDeg, GpsWgsMeasurement::kMaxLatitudeDeg);
  EXPECT_DEATH(
      GpsWgsMeasurement(
          sensor_id, timestamp_nanoseconds, latidude_deg, longitude_deg,
          altitude_meters),
      "");
  longitude_deg = getRandomUniformDouble(
      GpsWgsMeasurement::kMinLongitudeDeg, GpsWgsMeasurement::kMaxLongitudeDeg);
  EXPECT_DEATH(
      GpsWgsMeasurement(
          sensor_id, timestamp_nanoseconds, latidude_deg, longitude_deg,
          altitude_meters),
      "");
  altitude_meters = getRandomUniformDouble(
      GpsWgsMeasurement::kMinAltitudeMeters,
      GpsWgsMeasurement::kMaxAltitudeMeters);
  // Testing construction with valid GPS values.
  gps_wgs_measurement = GpsWgsMeasurement(
      sensor_id, timestamp_nanoseconds, latidude_deg, longitude_deg,
      altitude_meters);

  timestamp_nanoseconds = getRandomTimestampNanoseconds();
  gps_wgs_measurement = GpsWgsMeasurement(
      sensor_id, timestamp_nanoseconds, latidude_deg, longitude_deg,
      altitude_meters);

  // Testing timestamp and GPS values getters.
  EXPECT_EQ(
      gps_wgs_measurement.getTimestampNanoseconds(), timestamp_nanoseconds);
  EXPECT_EQ(gps_wgs_measurement.getAltitudeMeters(), altitude_meters);
  EXPECT_EQ(gps_wgs_measurement.getLatitudeDeg(), latidude_deg);
  EXPECT_EQ(gps_wgs_measurement.getLongitudeDeg(), longitude_deg);
}

TEST_F(MeasurementsTest, TestAccessorsRelative6DoFPoseMeasurement) {
  // Testing construction with invalid sensor id and invalid timestamp.
  SensorId sensor_id;
  int64_t timestamp_nanoseconds = -1;
  aslam::Transformation T_R_Sk;
  T_R_Sk.setRandom();
  const aslam::TransformationCovariance measurement_covariance =
      aslam::TransformationCovariance::Random();
  EXPECT_DEATH(
      Relative6DoFPoseMeasurement(
          sensor_id, timestamp_nanoseconds, T_R_Sk, measurement_covariance),
      "");
  common::generateId(&sensor_id);
  EXPECT_DEATH(
      Relative6DoFPoseMeasurement(
          sensor_id, timestamp_nanoseconds, T_R_Sk, measurement_covariance),
      "");

  timestamp_nanoseconds = getRandomTimestampNanoseconds();
  Relative6DoFPoseMeasurement rel_6dof_pose_measurement(
      sensor_id, timestamp_nanoseconds, T_R_Sk, measurement_covariance);

  // Testing T_R_Sk and measurement covariance getters.
  EXPECT_EQ(rel_6dof_pose_measurement.get_T_R_Sk(), T_R_Sk);
  EXPECT_EQ(
      rel_6dof_pose_measurement.getMeasurementCovariance(),
      measurement_covariance);
}
}  // namespace vi_map

MAPLAB_UNITTEST_ENTRYPOINT
