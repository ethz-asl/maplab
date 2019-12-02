#ifndef SENSORS_LOOP_CLOSURE_SENSOR_H_
#define SENSORS_LOOP_CLOSURE_SENSOR_H_

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

class LoopClosureSensor final : public aslam::Sensor {
 public:
  MAPLAB_POINTER_TYPEDEFS(LoopClosureSensor);

  LoopClosureSensor();
  explicit LoopClosureSensor(
      const aslam::SensorId& sensor_id, const std::string& topic);

  Sensor::Ptr cloneAsSensor() const {
    return std::dynamic_pointer_cast<Sensor>(
        aligned_shared<LoopClosureSensor>(*this));
  }

  LoopClosureSensor* cloneWithNewIds() const {
    LoopClosureSensor* cloned_relative_sensor = new LoopClosureSensor();
    *cloned_relative_sensor = *this;
    aslam::SensorId new_id;
    aslam::generateId(&new_id);
    cloned_relative_sensor->setId(new_id);
    return cloned_relative_sensor;
  }

  uint8_t getSensorType() const override {
    return SensorType::kLoopClosureSensor;
  }

  std::string getSensorTypeString() const override {
    return static_cast<std::string>(kLoopClosureSensorIdentifier);
  }

  inline bool get_T_A_B_fixed_covariance(
      aslam::TransformationCovariance* T_A_B_fixed_covariance) const {
    if (has_fixed_T_A_B_covariance_) {
      *T_A_B_fixed_covariance = T_A_B_fixed_covariance_;
      return true;
    }
    return false;
  }

  inline void set_T_A_B_fixed_covariance(
      const aslam::TransformationCovariance& T_A_B_fixed_covariance) {
    T_A_B_fixed_covariance_ = T_A_B_fixed_covariance;
    has_fixed_T_A_B_covariance_ = true;
  }

  inline bool hasFixedT_A_B_covariance() {
    return has_fixed_T_A_B_covariance_;
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

  bool has_fixed_T_A_B_covariance_;
  // Can be set as a property of the sensor and be used as a fall-back in case
  // the measurements do not come with individual covariances.
  aslam::TransformationCovariance T_A_B_fixed_covariance_;
};

class LoopClosureMeasurement : public Measurement {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  MAPLAB_POINTER_TYPEDEFS(LoopClosureMeasurement);

  LoopClosureMeasurement() {
    timestamp_nanoseconds_B_ = aslam::time::getInvalidTime();
    static_cast<Measurement*>(this)->setTimestampNanoseconds(
        aslam::time::getInvalidTime());
    T_A_B_.setIdentity();
    T_A_B_covariance_.setZero();
  }

  explicit LoopClosureMeasurement(
      const aslam::SensorId& sensor_id, const int64_t timestamp_nanoseconds_A,
      const int64_t timestamp_nanoseconds_B, const aslam::Transformation& T_A_B,
      const aslam::TransformationCovariance& covariance)
      : Measurement(sensor_id, timestamp_nanoseconds_A),
        T_A_B_(T_A_B),
        timestamp_nanoseconds_B_(timestamp_nanoseconds_B),
        T_A_B_covariance_(covariance) {}

  const aslam::Transformation& get_T_A_B() const {
    return T_A_B_;
  }

  int64_t getTimestampNanosecondsB() const {
    return timestamp_nanoseconds_B_;
  }

  int64_t* getTimestampNanosecondsBMutable() {
    return &timestamp_nanoseconds_B_;
  }

  void setTimestampNanosecondsB(const int64_t timestamp_nanoseconds_B) {
    timestamp_nanoseconds_B_ = timestamp_nanoseconds_B;
  }

  int64_t getTimestampNanosecondsA() const {
    return static_cast<Measurement const*>(this)->getTimestampNanoseconds();
  }

  int64_t* getTimestampNanosecondsAMutable() {
    return static_cast<Measurement*>(this)->getTimestampNanosecondsMutable();
  }

  void setTimestampNanosecondsA(const int64_t timestamp_nanoseconds_A) {
    static_cast<Measurement*>(this)->setTimestampNanoseconds(
        timestamp_nanoseconds_A);
  }

  int64_t getTimestampNanoseconds() const = delete;
  int64_t* getTimestampNanosecondsMutable() = delete;
  void setTimestampNanoseconds(const int64_t timestamp_nanoseconds) = delete;

  void set_T_A_B(const aslam::Transformation& T_A_B) {
    T_A_B_ = T_A_B;
  }

  const aslam::TransformationCovariance& get_T_A_B_covariance() const {
    return T_A_B_covariance_;
  }

  void set_T_A_B_covariance(
      const aslam::TransformationCovariance& measurement_covariance) {
    T_A_B_covariance_ = measurement_covariance;
  }

 private:
  bool isValidImpl() const {
    bool valid = true;
    valid &= aslam::time::isValidTime(timestamp_nanoseconds_B_);
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
    timestamp_nanoseconds_B_ = timestamp_distribution(random_engine);

    T_A_B_.setRandom();
    T_A_B_covariance_.setRandom();
  }

  aslam::Transformation T_A_B_;
  int64_t timestamp_nanoseconds_B_;
  aslam::TransformationCovariance T_A_B_covariance_;
};

DEFINE_MEAUREMENT_CONTAINERS(LoopClosureMeasurement)

// Hash function to buffer loop closure edges in a map based on the timestamps
// of the connected frames.
struct LoopClosureTimestampPairHasher {
  size_t operator()(const std::pair<int64_t, int64_t>& timestamp_pair) const {
    return static_cast<size_t>(timestamp_pair.first ^ timestamp_pair.second);
  }
};
typedef std::unordered_map<
    std::pair<int64_t, int64_t>, vi_map::LoopClosureMeasurement::ConstPtr,
    LoopClosureTimestampPairHasher>
    LoopClosureTemporalMap;

}  // namespace vi_map

DEFINE_MEASUREMENT_HASH(LoopClosureMeasurement)

#endif  // SENSORS_LOOP_CLOSURE_SENSOR_H_
