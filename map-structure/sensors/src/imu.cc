#include "sensors/imu.h"

#include <aslam/common/yaml-serialization.h>
#include <maplab-common/gravity-provider.h>

// The default values are for an ADIS16448.
DEFINE_double(
    gyro_noise_density, 1e-4, "Gyro noise density. [rad/s*1/sqrt(Hz)]");
DEFINE_double(
    gyro_bias_random_walk, 1e-4,
    "Gyro bias random  walk. [rad/s^2*1/sqrt(Hz)]");
DEFINE_double(
    acc_noise_density, 4e-3, "Accelerometer noise density. [m/s^2*1/sqrt(Hz)]");
DEFINE_double(
    acc_bias_random_walk, 4e-3,
    "Accelerometer bias random walk. [m/s^3*1/sqrt(Hz)]");
DEFINE_double(
    saturation_accel_max, 150.0, "Saturation of accelerometer [m/s^2]");
DEFINE_double(saturation_gyro_max, 7.5, "Saturation of gyroscope [rad/s]");

constexpr char kYamlFieldNameSigmas[] = "sigmas";
constexpr char kYamlFieldNameSaturationAccelMaxMps2[] =
    "saturation_accel_max_mps2";
constexpr char kYamlFieldNameSaturationGyroMaxRadps[] =
    "saturation_gyro_max_radps";
constexpr char kYamlFieldNameGravityMagnitudeMps2[] = "gravity_magnitude_mps2";
constexpr char kYamlFieldNameGyroNoiseDensity[] = "gyro_noise_density";
constexpr char kYamlFieldNameGyroBiasRandomWalkNoiseDensity[] =
    "gyro_bias_random_walk_noise_density";
constexpr char kYamlFieldNameAccelNoiseDensity[] = "acc_noise_density";
constexpr char kYamlFieldNameAccelBiasRandomWalkNoiseDensity[] =
    "acc_bias_random_walk_noise_density";

namespace vi_map {

constexpr double Imu::kMinGravity;
constexpr double Imu::kMaxGravity;
constexpr double Imu::kMinSaturationAccelMax;
constexpr double Imu::kMinSaturationGyroMax;

template <>
SensorType sensorToType<Imu>() {
  return SensorType::kImu;
}

constexpr char kDefaultImuHardwareId[] = "imu";

ImuSigmas::ImuSigmas(
    const double _gyro_noise_density, const double _gyro_bias_noise_density,
    const double _acc_noise_density, const double _acc_bias_noise_density)
    : gyro_noise_density(_gyro_noise_density),
      gyro_bias_random_walk_noise_density(_gyro_bias_noise_density),
      acc_noise_density(_acc_noise_density),
      acc_bias_random_walk_noise_density(_acc_bias_noise_density) {}

ImuSigmas::ImuSigmas()
    : gyro_noise_density(FLAGS_gyro_noise_density),
      gyro_bias_random_walk_noise_density(FLAGS_gyro_bias_random_walk),
      acc_noise_density(FLAGS_acc_noise_density),
      acc_bias_random_walk_noise_density(FLAGS_acc_bias_random_walk) {
  CHECK(isValid());
}

Imu::Imu()
    : Sensor(SensorType::kImu, static_cast<std::string>(kDefaultImuHardwareId)),
      saturation_accel_max_(FLAGS_saturation_accel_max),
      saturation_gyro_max_radps_(FLAGS_saturation_gyro_max) {
  common::GravityProvider gravity_provider(
      common::locations::kAltitudeZurichMeters,
      common::locations::kLatitudeZurichDegrees);
  gravity_magnitude_ = gravity_provider.getGravityMagnitude();
  CHECK(isValid());
}

Imu::Imu(const SensorId& sensor_id, const std::string& hardware_id)
    : Sensor(sensor_id, SensorType::kImu, hardware_id),
      saturation_accel_max_(FLAGS_saturation_accel_max),
      saturation_gyro_max_radps_(FLAGS_saturation_gyro_max) {
  common::GravityProvider gravity_provider(
      common::locations::kAltitudeZurichMeters,
      common::locations::kLatitudeZurichDegrees);
  gravity_magnitude_ = gravity_provider.getGravityMagnitude();
  CHECK(isValid());
}

bool Imu::loadFromYamlNodeImpl(const YAML::Node& sensor_node) {
  if (!YAML::safeGet(
          sensor_node, static_cast<std::string>(kYamlFieldNameSigmas),
          &sigmas_)) {
    LOG(ERROR) << "Unable to find the IMU sigmas.";
    return false;
  }

  if (!YAML::safeGet(
          sensor_node,
          static_cast<std::string>(kYamlFieldNameSaturationAccelMaxMps2),
          &saturation_accel_max_)) {
    LOG(ERROR) << "Unable to find the IMU acceleration saturation max value.";
    return false;
  }

  if (!YAML::safeGet(
          sensor_node,
          static_cast<std::string>(kYamlFieldNameSaturationGyroMaxRadps),
          &saturation_gyro_max_radps_)) {
    LOG(ERROR) << "Unable to find the IMU gyro saturation max value.";
    return false;
  }

  if (!YAML::safeGet(
          sensor_node,
          static_cast<std::string>(kYamlFieldNameGravityMagnitudeMps2),
          &gravity_magnitude_)) {
    LOG(ERROR) << "Unable to find the gravity magnitude.";
    return false;
  }
  return isValid();
}

void Imu::saveToYamlNodeImpl(YAML::Node* sensor_node) const {
  CHECK_NOTNULL(sensor_node);
  YAML::Node& node = *sensor_node;
  node[static_cast<std::string>(kYamlFieldNameSigmas)] = sigmas_;
  node[static_cast<std::string>(kYamlFieldNameSaturationAccelMaxMps2)] =
      saturation_accel_max_;
  node[static_cast<std::string>(kYamlFieldNameSaturationGyroMaxRadps)] =
      saturation_gyro_max_radps_;
  node[static_cast<std::string>(kYamlFieldNameGravityMagnitudeMps2)] =
      gravity_magnitude_;
}

bool ImuSigmas::loadFromYaml(const std::string& yaml_path) {
  return YAML::Load(yaml_path, this);
}

bool ImuSigmas::saveToYaml(const std::string& yaml_path) const {
  return YAML::Save(*this, yaml_path);
}

void Imu::setRandomImpl() {
  sigmas_.setRandom();

  std::random_device random_device;
  std::default_random_engine random_engine(random_device());
  std::uniform_real_distribution<double> uniform_gravity(
      kMinGravity, kMaxGravity);
  gravity_magnitude_ = uniform_gravity(random_engine);

  constexpr double kMinSaturation = 0.0;
  constexpr double kMaxSaturation = 1e3;
  std::uniform_real_distribution<double> saturation(
      kMinSaturation, kMaxSaturation);
  saturation_accel_max_ = saturation(random_engine);
  saturation_gyro_max_radps_ = saturation(random_engine);
}

void ImuSigmas::setRandom() {
  std::random_device random_device;
  std::default_random_engine random_engine(random_device());

  constexpr double kMinNoiseDensity = 0.01;
  constexpr double kMaxNoiseDensity = 1e3;
  std::uniform_real_distribution<double> noise_density(
      kMinNoiseDensity, kMaxNoiseDensity);
  gyro_noise_density = noise_density(random_engine);
  acc_noise_density = noise_density(random_engine);

  constexpr double kMinRandomWalkNoiseDensity = 0.01;
  constexpr double kMaxRandomWalkNoiseDensity = 1e3;
  std::uniform_real_distribution<double> random_walk(
      kMinRandomWalkNoiseDensity, kMaxRandomWalkNoiseDensity);
  gyro_bias_random_walk_noise_density = noise_density(random_engine);
  acc_bias_random_walk_noise_density = noise_density(random_engine);
}

}  // namespace vi_map

namespace YAML {
Node convert<vi_map::ImuSigmas>::encode(const vi_map::ImuSigmas& sigmas) {
  Node sigmas_node;
  sigmas_node[static_cast<std::string>(kYamlFieldNameGyroNoiseDensity)] =
      sigmas.gyro_noise_density;
  sigmas_node[static_cast<std::string>(
      kYamlFieldNameGyroBiasRandomWalkNoiseDensity)] =
      sigmas.gyro_bias_random_walk_noise_density;
  sigmas_node[static_cast<std::string>(kYamlFieldNameAccelNoiseDensity)] =
      sigmas.acc_noise_density;
  sigmas_node[static_cast<std::string>(
      kYamlFieldNameAccelBiasRandomWalkNoiseDensity)] =
      sigmas.acc_bias_random_walk_noise_density;
  return sigmas_node;
}

bool convert<vi_map::ImuSigmas>::decode(
    const Node& node, vi_map::ImuSigmas& config) {  // NOLINT
  bool success = true;
  success &= YAML::safeGet(
      node, static_cast<std::string>(kYamlFieldNameGyroNoiseDensity),
      &config.gyro_noise_density);
  success &= YAML::safeGet(
      node,
      static_cast<std::string>(kYamlFieldNameGyroBiasRandomWalkNoiseDensity),
      &config.gyro_bias_random_walk_noise_density);
  success &= YAML::safeGet(
      node, static_cast<std::string>(kYamlFieldNameAccelNoiseDensity),
      &config.acc_noise_density);
  success &= YAML::safeGet(
      node,
      static_cast<std::string>(kYamlFieldNameAccelBiasRandomWalkNoiseDensity),
      &config.acc_bias_random_walk_noise_density);
  return success;
}

}  // namespace YAML
