#ifndef SENSORS_WHEEL_ODOMETRY_SENSOR_H_
#define SENSORS_WHEEL_ODOMETRY_SENSOR_H_

#include <string>
#include <unordered_map>
#include <utility>

#include <aslam/common/sensor.h>
#include <aslam/common/time.h>
#include <maplab-common/macros.h>
#include <yaml-cpp/yaml.h>

#include "sensors/measurement.h"
#include "sensors/sensor-types.h"

namespace vi_map {

class WheelOdometry final : public aslam::Sensor {
 public:
  MAPLAB_POINTER_TYPEDEFS(WheelOdometry);

  WheelOdometry();
  explicit WheelOdometry(
      const aslam::SensorId& sensor_id, const std::string& topic);

  Sensor::Ptr cloneAsSensor() const {
    return std::dynamic_pointer_cast<Sensor>(
        aligned_shared<WheelOdometry>(*this));
  }

  WheelOdometry* cloneWithNewIds() const {
    WheelOdometry* cloned_relative_sensor = new WheelOdometry();
    *cloned_relative_sensor = *this;
    aslam::SensorId new_id;
    aslam::generateId(&new_id);
    cloned_relative_sensor->setId(new_id);
    return cloned_relative_sensor;
  }

  uint8_t getSensorType() const override {
    return SensorType::kWheelOdometry;
  }

  std::string getSensorTypeString() const override {
    return static_cast<std::string>(kWheelOdometryIdentifier);
  }

  inline bool get_T_St_stp1_fixed_covariance(
      aslam::TransformationCovariance* T_St_stp1_fixed_covariance) const {
    if (has_fixed_T_St_Stp1_fixed_covariance_) {
      *T_St_stp1_fixed_covariance = T_St_Stp1_fixed_covariance_;
      return true;
    }
    return false;
  }

  inline void set_T_St_stp1_fixed_covariance(
      const aslam::TransformationCovariance& T_St_stp1_fixed_covariance) {
    T_St_Stp1_fixed_covariance_ = T_St_stp1_fixed_covariance;
    has_fixed_T_St_Stp1_fixed_covariance_ = true;
  }

  inline bool hasFixedT_St_Stp1_covariance() {
    return has_fixed_T_St_Stp1_fixed_covariance_;
  }

 private:
  bool loadFromYamlNodeImpl(const YAML::Node& sensor_node) override;
  void saveToYamlNodeImpl(YAML::Node* sensor_node) const override;

  bool isValidImpl() const override {
    return true;
  }

  void setRandomImpl() override {}

  bool isEqualImpl(
      const Sensor& /*other*/, const bool /*verbose*/) const override {
    return true;
  }

  bool has_fixed_T_St_Stp1_fixed_covariance_;

  // Can be set as a property of the sensor and be used as a fall-back in case
  // the measurements do not come with individual covariances.
  //
  // NOTE: Currently this is not used as the covariance of one measurement, but
  // as the fixed covariance added between two vertices that are connected by a
  // wheel odometry edge, independently of how many measurements occured in
  // between.
  aslam::TransformationCovariance T_St_Stp1_fixed_covariance_;
};

class WheelOdometryMeasurement : public Measurement {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  MAPLAB_POINTER_TYPEDEFS(WheelOdometryMeasurement);

  WheelOdometryMeasurement() {
    setTimestampNanoseconds(aslam::time::getInvalidTime());
    T_S0_St_.setIdentity();
    T_St_Stp1_fixed_covariance_.setZero();
  }

  explicit WheelOdometryMeasurement(
      const aslam::SensorId& sensor_id, const int64_t timestamp_nanoseconds,
      const aslam::Transformation& T_S0_St,
      const aslam::TransformationCovariance& covariance)
      : Measurement(sensor_id, timestamp_nanoseconds),
        T_S0_St_(T_S0_St),
        T_St_Stp1_fixed_covariance_(covariance) {}

  const aslam::Transformation& get_T_S0_St() const {
    return T_S0_St_;
  }

  void set_T_S0_St(const aslam::Transformation& T_S0_St) {
    T_S0_St_ = T_S0_St;
  }

  const aslam::TransformationCovariance& get_T_St_Stp1_covariance() const {
    return T_St_Stp1_fixed_covariance_;
  }

  void set_T_St_Stp1_covariance(
      const aslam::TransformationCovariance& measurement_covariance) {
    T_St_Stp1_fixed_covariance_ = measurement_covariance;
  }

 private:
  bool isValidImpl() const {
    bool valid = true;
    valid &= aslam::time::isValidTime(
        static_cast<Measurement const*>(this)->getTimestampNanoseconds());
    return valid;
  }

  void setRandomImpl() override {
    std::random_device random_device;
    std::default_random_engine random_engine(random_device());
    constexpr int64_t kMinTimestampNanoseconds = 0;
    constexpr int64_t kMaxTimestampNanoseconds = static_cast<int64_t>(1e14);
    std::uniform_int_distribution<int64_t> timestamp_distribution(
        kMinTimestampNanoseconds, kMaxTimestampNanoseconds);
    setTimestampNanoseconds(timestamp_distribution(random_engine));

    T_S0_St_.setRandom();
    T_St_Stp1_fixed_covariance_.setRandom();
  }

  // the wheel odometry sensor is assumed to measure the transformation from the
  // initial pose at time 0 to the current pose at time t.
  aslam::Transformation T_S0_St_;
  // T_St_Stp1_covariance refers to the covariance of the relative measurement
  // between two subsequent measurements. Currently only the sensor covariance
  // is used.
  aslam::TransformationCovariance T_St_Stp1_fixed_covariance_;
};

DEFINE_MEAUREMENT_CONTAINERS(WheelOdometryMeasurement)
}  // namespace vi_map

DEFINE_MEASUREMENT_HASH(WheelOdometryMeasurement)

#endif  // SENSORS_WHEEL_ODOMETRY_SENSOR_H_
