#include <random>

#include <gtest/gtest.h>
#include <maplab-common/test/testing-entrypoint.h>
#include <maplab-common/test/testing-predicates.h>

#include "sensors/gps-utm.h"
#include "sensors/gps-wgs.h"
#include "sensors/imu.h"
#include "sensors/lidar.h"
#include "sensors/loop-closure-sensor.h"
#include "sensors/wheel-odometry-sensor.h"

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
  aslam::SensorId sensor_id;
  CHECK(!sensor_id.isValid());
  int64_t timestamp_nanoseconds = 0;
  EXPECT_DEATH(
      GpsUtmMeasurement(
          sensor_id, timestamp_nanoseconds, aslam::Transformation(),
          UtmZone(32u, 'N')),
      "sensor_id_.isValid()");
  timestamp_nanoseconds = -1;

  aslam::generateId(&sensor_id);

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
  GpsUtmMeasurement gps_utm_measurement = GpsUtmMeasurement(
      sensor_id, timestamp_nanoseconds, T_UTM_S, UtmZone::createInvalid());
  EXPECT_EQ(
      gps_utm_measurement.getTimestampNanoseconds(), timestamp_nanoseconds);
  EXPECT_EQ(gps_utm_measurement.get_T_UTM_S(), T_UTM_S);
  EXPECT_EQ(gps_utm_measurement.getSensorId(), sensor_id);
}

TEST_F(MeasurementsTest, TestAccessorsGpsWgs) {
  aslam::SensorId sensor_id;
  aslam::generateId(&sensor_id);

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
  GpsWgsMeasurement gps_wgs_measurement = GpsWgsMeasurement(
      sensor_id, timestamp_nanoseconds, latidude_deg, longitude_deg,
      altitude_meters);

  timestamp_nanoseconds = getRandomTimestampNanoseconds();
  gps_wgs_measurement = GpsWgsMeasurement(
      sensor_id, timestamp_nanoseconds, latidude_deg, longitude_deg,
      altitude_meters);
  EXPECT_EQ(gps_wgs_measurement.getSensorId(), sensor_id);

  // Testing timestamp and GPS values getters.
  EXPECT_EQ(
      gps_wgs_measurement.getTimestampNanoseconds(), timestamp_nanoseconds);
  EXPECT_EQ(gps_wgs_measurement.getAltitudeMeters(), altitude_meters);
  EXPECT_EQ(gps_wgs_measurement.getLatitudeDeg(), latidude_deg);
  EXPECT_EQ(gps_wgs_measurement.getLongitudeDeg(), longitude_deg);
}

TEST_F(MeasurementsTest, TestAccessorsLoopClosureMeasurement) {
  // Testing construction with invalid sensor id and invalid timestamp.
  aslam::SensorId sensor_id;
  int64_t timestamp_ns_A = aslam::time::getInvalidTime();
  int64_t timestamp_ns_B = aslam::time::getInvalidTime();
  aslam::Transformation T_A_B;
  T_A_B.setRandom();
  const aslam::TransformationCovariance measurement_covariance =
      aslam::TransformationCovariance::Random();
  EXPECT_DEATH(
      LoopClosureMeasurement(
          sensor_id, timestamp_ns_A, timestamp_ns_B, T_A_B,
          measurement_covariance),
      "");
  aslam::generateId(&sensor_id);
  EXPECT_DEATH(
      LoopClosureMeasurement(
          sensor_id, timestamp_ns_A, timestamp_ns_B, T_A_B,
          measurement_covariance),
      "");

  timestamp_ns_A = getRandomTimestampNanoseconds();
  timestamp_ns_B = getRandomTimestampNanoseconds();
  LoopClosureMeasurement loop_closure_measurement(
      sensor_id, timestamp_ns_A, timestamp_ns_B, T_A_B, measurement_covariance);

  // Testing T_A_B and measurement covariance getters.
  EXPECT_EQ(loop_closure_measurement.get_T_A_B(), T_A_B);
  EXPECT_EQ(
      loop_closure_measurement.get_T_A_B_covariance(), measurement_covariance);
  EXPECT_EQ(
      loop_closure_measurement.getTimestampNanosecondsA(), timestamp_ns_A);
  EXPECT_EQ(
      loop_closure_measurement.getTimestampNanosecondsB(), timestamp_ns_B);
}

TEST_F(MeasurementsTest, TestAccessorsLidarMeasurement) {
  // Testing construction with invalid sensor id and invalid timestamp.
  aslam::SensorId sensor_id;
  int64_t timestamp_nanoseconds = -1;
  EXPECT_DEATH(MaplabLidarMeasurement(sensor_id, timestamp_nanoseconds), "");
  aslam::generateId(&sensor_id);
  EXPECT_DEATH(MaplabLidarMeasurement(sensor_id, timestamp_nanoseconds), "");

  timestamp_nanoseconds = getRandomTimestampNanoseconds();
  MaplabLidarMeasurement lidar_measurement(sensor_id, timestamp_nanoseconds);
  EXPECT_TRUE(lidar_measurement.getPointCloud().empty());
}

TEST_F(MeasurementsTest, TestAccessorsImuMeasurement) {
  // Testing construction with invalid sensor id and invalid timestamp.
  const Eigen::Vector3d I_accel_xyz_m_s2 = Eigen::Vector3d::Random();
  const Eigen::Vector3d I_gyro_xyz_rad_s = Eigen::Vector3d::Random();
  aslam::SensorId sensor_id;
  int64_t timestamp_nanoseconds = -1;
  EXPECT_DEATH(
      ImuMeasurement(
          sensor_id, timestamp_nanoseconds, I_accel_xyz_m_s2, I_gyro_xyz_rad_s),
      "");
  aslam::generateId(&sensor_id);
  EXPECT_DEATH(
      ImuMeasurement(
          sensor_id, timestamp_nanoseconds, I_accel_xyz_m_s2, I_gyro_xyz_rad_s),
      "");

  timestamp_nanoseconds = getRandomTimestampNanoseconds();

  const ImuMeasurement imu_measurement(
      sensor_id, timestamp_nanoseconds, I_accel_xyz_m_s2, I_gyro_xyz_rad_s);
  EXPECT_EQ(imu_measurement.get_I_Accel_xyz_m_s2(), I_accel_xyz_m_s2);
  EXPECT_EQ(imu_measurement.get_I_Gyro_xyz_rad_s(), I_gyro_xyz_rad_s);
}

}  // namespace vi_map

MAPLAB_UNITTEST_ENTRYPOINT
