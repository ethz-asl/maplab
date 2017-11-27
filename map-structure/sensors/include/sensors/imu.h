#ifndef SENSORS_IMU_H_
#define SENSORS_IMU_H_

#include <random>
#include <string>

#include <maplab-common/macros.h>
#include <yaml-cpp/yaml.h>

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
}  // namespace vi_map

namespace YAML {
template <>
struct convert<vi_map::ImuSigmas> {
  static bool decode(const Node& node, vi_map::ImuSigmas& camera);  // NOLINT
  static Node encode(const vi_map::ImuSigmas& camera);
};
}  // namespace YAML

#endif  // SENSORS_IMU_H_
