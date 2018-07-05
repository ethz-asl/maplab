#include "sensors/imu.h"

#include <aslam/common/yaml-serialization.h>
#include <maplab-common/eigen-proto.h>
#include <maplab-common/gravity-provider.h>

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

constexpr double kDefaultGyroNoiseDensity = 1e-4;
constexpr double kDefaultGyroBiasRandomWalk = 1e-4;
constexpr double kDefaultAccNoiseDensity = 4e-3;
constexpr double kDefaultAccBiasRandomWalk = 4e-3;

ImuSigmas::ImuSigmas()
    : gyro_noise_density(kDefaultGyroNoiseDensity),
      gyro_bias_random_walk_noise_density(kDefaultGyroBiasRandomWalk),
      acc_noise_density(kDefaultAccNoiseDensity),
      acc_bias_random_walk_noise_density(kDefaultAccBiasRandomWalk) {
  CHECK(isValid());
}

constexpr char kDefaultImuHardwareId[] = "imu";

ImuSigmas::ImuSigmas(
    const double _gyro_noise_density, const double _gyro_bias_noise_density,
    const double _acc_noise_density, const double _acc_bias_noise_density)
    : gyro_noise_density(_gyro_noise_density),
      gyro_bias_random_walk_noise_density(_gyro_bias_noise_density),
      acc_noise_density(_acc_noise_density),
      acc_bias_random_walk_noise_density(_acc_bias_noise_density) {}

constexpr double kDefaultSaturationAccelMax = 150.0;
constexpr double kDefaultSaturationGyroMax = 7.5;

Imu::Imu()
    : Sensor(SensorType::kImu, static_cast<std::string>(kDefaultImuHardwareId)),
      saturation_accel_max_(kDefaultSaturationAccelMax),
      saturation_gyro_max_radps_(kDefaultSaturationGyroMax) {
  common::GravityProvider gravity_provider(
      common::locations::kAltitudeZurichMeters,
      common::locations::kLatitudeZurichDegrees);
  gravity_magnitude_ = gravity_provider.getGravityMagnitude();
  CHECK(isValid());
}

Imu::Imu(const SensorId& sensor_id, const std::string& hardware_id)
    : Sensor(sensor_id, SensorType::kImu, hardware_id),
      saturation_accel_max_(kDefaultSaturationAccelMax),
      saturation_gyro_max_radps_(kDefaultSaturationGyroMax) {
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

void ImuMeasurement::serialize(
    measurements::proto::ImuMeasurement* proto_measurement) const {
  CHECK_NOTNULL(proto_measurement);
  Measurement::serialize(proto_measurement->mutable_measurement_base());
  common::eigen_proto::serialize(
      I_accel_xyz_m_s2_gyro_xyz_rad_s_,
      proto_measurement->mutable_i_accel_xyz_m_s2_gyro_xyz_rad_s());
}

void ImuMeasurement::deserialize(
    const measurements::proto::ImuMeasurement& proto_measurement) {
  Measurement::deserialize(proto_measurement.measurement_base());
  common::eigen_proto::deserialize(
      proto_measurement.i_accel_xyz_m_s2_gyro_xyz_rad_s(),
      &I_accel_xyz_m_s2_gyro_xyz_rad_s_);
}

void ImuMeasurement::setRandomImpl() {
  I_accel_xyz_m_s2_gyro_xyz_rad_s_.setRandom();
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

namespace common {
template <>
void linearInterpolation<int64_t, vi_map::ImuMeasurement>(
    const int64_t t1, const vi_map::ImuMeasurement& x1, const int64_t t2,
    const vi_map::ImuMeasurement& x2, const int64_t t_interpolated,
    vi_map::ImuMeasurement* x_interpolated) {
  CHECK_NOTNULL(x_interpolated);
  CHECK_LE(t1, t2);
  CHECK_LE(t1, t_interpolated);
  CHECK_LE(t_interpolated, t2);
  CHECK(x1.getSensorId().isValid());
  CHECK_EQ(x1.getSensorId(), x2.getSensorId());
  if (t1 == t2) {
    CHECK(x1 == x2);
    *x_interpolated = x1;
    return;
  }

  vi_map::ImuMeasurement::CombinedImuData I_accel_m_s2_gyro_rad_s_interpolated;
  linearInterpolation(
      t1, x1.get_I_Accel_m_s2_Gyro_rad_s(), t2,
      x2.get_I_Accel_m_s2_Gyro_rad_s(), t_interpolated,
      &I_accel_m_s2_gyro_rad_s_interpolated);

  *x_interpolated = vi_map::ImuMeasurement(
      x1.getSensorId(), t_interpolated, I_accel_m_s2_gyro_rad_s_interpolated);
}
}  // namespace common
