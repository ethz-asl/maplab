#ifndef SENSORS_IMU_H_
#define SENSORS_IMU_H_

#include <random>
#include <string>

#include <maplab-common/interpolation-helpers.h>
#include <maplab-common/macros.h>
#include <yaml-cpp/yaml.h>

#include "sensors/measurement.h"
#include "sensors/measurements.pb.h"
#include "sensors/sensor.h"

namespace vi_map {

struct ImuSigmas {
  ImuSigmas();
  ImuSigmas(
      const double _gyro_noise_density, const double _gyro_bias_noise_density,
      const double _acc_noise_density, const double _acc_bias_noise_density);
  // Gyro noise density. [rad/s*1/sqrt(Hz)]
  double gyro_noise_density;
  /// Gyro bias random walk. [rad/s^2*1/sqrt(Hz)]
  double gyro_bias_random_walk_noise_density;
  // Accelerometer noise density. [m/s^2*1/sqrt(Hz)]
  double acc_noise_density;
  /// Accelerometer bias random walk. [m/s^3*1/sqrt(Hz)]
  double acc_bias_random_walk_noise_density;

  bool isValid() const {
    return gyro_noise_density > 0. &&
           gyro_bias_random_walk_noise_density > 0. && acc_noise_density > 0. &&
           acc_bias_random_walk_noise_density > 0.;
  }

  bool operator==(const ImuSigmas& other) const {
    return gyro_noise_density == other.gyro_noise_density &&
           gyro_bias_random_walk_noise_density ==
               other.gyro_bias_random_walk_noise_density &&
           acc_noise_density == other.acc_noise_density &&
           acc_bias_random_walk_noise_density ==
               other.acc_bias_random_walk_noise_density;
  }

  bool isEqual(const ImuSigmas& other, const double precision) const {
    return std::abs(gyro_noise_density - other.gyro_noise_density) <
               precision &&
           std::abs(
               gyro_bias_random_walk_noise_density -
               other.gyro_bias_random_walk_noise_density) < precision &&
           std::abs(acc_noise_density - other.acc_noise_density) < precision &&
           std::abs(
               acc_bias_random_walk_noise_density -
               other.acc_bias_random_walk_noise_density) < precision;
  }

  bool loadFromYaml(const std::string& yaml_path);
  bool saveToYaml(const std::string& yaml_path) const;

  void setRandom();
};

class Imu final : public Sensor {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  MAPLAB_POINTER_TYPEDEFS(Imu);
  Imu();
  Imu(const SensorId& sensor_id, const std::string& hardware_id);

  virtual ~Imu() = default;

  const ImuSigmas& getImuSigmas() const {
    return sigmas_;
  }

  void setImuSigmas(const ImuSigmas& sigmas) {
    sigmas_ = sigmas;
  }

  double getGravityMagnitudeMps2() const {
    return gravity_magnitude_;
  }

  void setGravityMagnitude(const double gravity_magnitude) {
    gravity_magnitude_ = gravity_magnitude;
    CHECK_GE(gravity_magnitude_, kMinGravity);
    CHECK_LE(gravity_magnitude_, kMaxGravity);
  }

  double getAccelerationSaturationMax() const {
    return saturation_accel_max_;
  }

  double getGyroSaturationMaxRadps() const {
    return saturation_gyro_max_radps_;
  }

  Sensor::UniquePtr clone() const override {
    return aligned_unique<Imu>(*this);
  }

  static constexpr double kMinGravity = 9.0;
  static constexpr double kMaxGravity = 10.0;
  static constexpr double kMinSaturationAccelMax = 0.0;
  static constexpr double kMinSaturationGyroMax = 0.0;

 private:
  bool loadFromYamlNodeImpl(const YAML::Node& sensor_node) override;
  void saveToYamlNodeImpl(YAML::Node* sensor_node) const override;

  bool isValidImpl() const override {
    RETURN_FALSE_IF_WRONG_SENSOR_TYPE(kImu);
    return sigmas_.isValid() && gravity_magnitude_ >= kMinGravity &&
           gravity_magnitude_ <= kMaxGravity &&
           saturation_accel_max_ >= kMinSaturationAccelMax &&
           saturation_gyro_max_radps_ >= kMinSaturationGyroMax;
  }

  bool isEqualImpl(const Sensor& other, const double precision) const override {
    const Imu& other_imu = static_cast<const Imu&>(other);
    return sigmas_.isEqual(other_imu.sigmas_, precision) &&
           std::abs(gravity_magnitude_ - other_imu.gravity_magnitude_) <
               precision &&
           std::abs(saturation_accel_max_ - other_imu.saturation_accel_max_) <
               precision &&
           std::abs(
               saturation_gyro_max_radps_ -
               other_imu.saturation_gyro_max_radps_) < precision;
  }

  void setRandomImpl() override;

  ImuSigmas sigmas_;
  double gravity_magnitude_;
  /// Accelerometer saturation[m/s^2]; 0.0 means undefined.
  double saturation_accel_max_;
  /// Gyroscope saturation. [rad/s]; 0.0 means undefined.
  double saturation_gyro_max_radps_;
};

class ImuMeasurement final : public Measurement {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  friend class MeasurementsTest_TestAccessorsImuMeasurement_Test;
  friend class MeasurementsTest_ImuMeasurementProtoSerialization_Test;
  typedef Eigen::Matrix<double, 6, 1> CombinedImuData;
  ImuMeasurement()
      : I_accel_xyz_m_s2_gyro_xyz_rad_s_(CombinedImuData::Zero()) {}
  ImuMeasurement(
      const SensorId& sensor_id, const int64_t timestamp_nanoseconds,
      const CombinedImuData& I_accel_m_s2_gyro_rad_s)
      : Measurement(sensor_id, timestamp_nanoseconds),
        I_accel_xyz_m_s2_gyro_xyz_rad_s_(I_accel_m_s2_gyro_rad_s) {
    CHECK(isValid());
  }
  ImuMeasurement(
      const SensorId& sensor_id, const int64_t timestamp_nanoseconds,
      const Eigen::Vector3d& I_accel_xyz_m_s2,
      const Eigen::Vector3d& I_gyro_xyz_rad_s)
      : Measurement(sensor_id, timestamp_nanoseconds) {
    I_accel_xyz_m_s2_gyro_xyz_rad_s_.topRows(3) = I_accel_xyz_m_s2;
    I_accel_xyz_m_s2_gyro_xyz_rad_s_.bottomRows(3) = I_gyro_xyz_rad_s;
    CHECK(isValid());
  }
  ~ImuMeasurement() override = default;

  bool operator==(const ImuMeasurement& other) const {
    return Measurement::operator==(other) &&
           I_accel_xyz_m_s2_gyro_xyz_rad_s_ ==
               other.get_I_Accel_m_s2_Gyro_rad_s();
  }

  const CombinedImuData& get_I_Accel_m_s2_Gyro_rad_s() const {
    return I_accel_xyz_m_s2_gyro_xyz_rad_s_;
  }

  Eigen::Map<const Eigen::Vector3d> get_I_Accel_xyz_m_s2() const {
    return Eigen::Map<const Eigen::Vector3d>(
        I_accel_xyz_m_s2_gyro_xyz_rad_s_.data(), 3);
  }

  Eigen::Map<const Eigen::Vector3d> get_I_Gyro_xyz_rad_s() const {
    return Eigen::Map<const Eigen::Vector3d>(
        I_accel_xyz_m_s2_gyro_xyz_rad_s_.data() + 3, 3);
  }

  void serialize(measurements::proto::ImuMeasurement* proto_measurement) const;
  void deserialize(
      const measurements::proto::ImuMeasurement& proto_measurement);

 private:
  explicit ImuMeasurement(const SensorId& sensor_id)
      : Measurement(sensor_id),
        I_accel_xyz_m_s2_gyro_xyz_rad_s_(CombinedImuData::Zero()) {}
  bool isValidImpl() const override {
    return true;
  }

  void setRandomImpl() override;

  CombinedImuData I_accel_xyz_m_s2_gyro_xyz_rad_s_;
};
DEFINE_MEAUREMENT_CONTAINERS(ImuMeasurement)

}  // namespace vi_map

DEFINE_MEASUREMENT_HASH(ImuMeasurement)

namespace YAML {
template <>
struct convert<vi_map::ImuSigmas> {
  static bool decode(const Node& node, vi_map::ImuSigmas& camera);  // NOLINT
  static Node encode(const vi_map::ImuSigmas& camera);
};
}  // namespace YAML

namespace common {
template <>
void linearInterpolation<int64_t, vi_map::ImuMeasurement>(
    const int64_t t1, const vi_map::ImuMeasurement& x1, const int64_t t2,
    const vi_map::ImuMeasurement& x2, const int64_t t_interpolated,
    vi_map::ImuMeasurement* x_interpolated);
}  // namespace common

#endif  // SENSORS_IMU_H_
