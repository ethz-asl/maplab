#ifndef SENSORS_ABSOLUTE_6DOF_POSE_H_
#define SENSORS_ABSOLUTE_6DOF_POSE_H_

#include <string>

#include <aslam/common/sensor.h>
#include <maplab-common/macros.h>
#include <yaml-cpp/yaml.h>

#include "sensors/measurement.h"
#include "sensors/sensor-types.h"

namespace vi_map {

// Represents a 6DoF Absolute pose sensor, that provides a transformation from
// the sensor frame to the world frame, which is assumed to be the mission
// frame.
class Absolute6DoF final : public aslam::Sensor {
 public:
  MAPLAB_POINTER_TYPEDEFS(Absolute6DoF);

  Absolute6DoF();
  explicit Absolute6DoF(
      const aslam::SensorId& sensor_id, const std::string& topic);

  Sensor::Ptr cloneAsSensor() const {
    return std::dynamic_pointer_cast<Sensor>(
        aligned_shared<Absolute6DoF>(*this));
  }

  Absolute6DoF* cloneWithNewIds() const {
    Absolute6DoF* cloned_absolute_sensor = new Absolute6DoF();
    *cloned_absolute_sensor = *this;
    aslam::SensorId new_id;
    aslam::generateId(&new_id);
    cloned_absolute_sensor->setId(new_id);
    return cloned_absolute_sensor;
  }

  uint8_t getSensorType() const override {
    return SensorType::kAbsolute6DoF;
  }

  std::string getSensorTypeString() const override {
    return static_cast<std::string>(kAbsolute6DoFIdentifier);
  }

  inline bool get_T_G_S_fixed_covariance(
      aslam::TransformationCovariance* T_G_S_fixed_covariance) const {
    if (has_fixed_T_G_S_covariance_) {
      *T_G_S_fixed_covariance = T_G_S_fixed_covariance_;
      return true;
    }
    return false;
  }

  inline void set_T_G_S_fixed_covariance(
      const aslam::TransformationCovariance& T_G_S_fixed_covariance) {
    T_G_S_fixed_covariance_ = T_G_S_fixed_covariance;
    has_fixed_T_G_S_covariance_ = true;
  }

  inline bool has_T_G_S_fixed_covariance() {
    return has_fixed_T_G_S_covariance_;
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

  bool has_fixed_T_G_S_covariance_;

  // Can be set as a property of the sensor and be used as a fall-back in case
  // the measurements do not come with individual covariances.
  aslam::TransformationCovariance T_G_S_fixed_covariance_;
};

// Stores 6DoF absolute measurements, i.e. a transformation from the
// sensor frame (S) to the world frame (W)
class Absolute6DoFMeasurement : public Measurement {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  MAPLAB_POINTER_TYPEDEFS(Absolute6DoFMeasurement);

  Absolute6DoFMeasurement() {
    T_G_S_.setIdentity();
    T_G_S_covariance_.setZero();
  }

  explicit Absolute6DoFMeasurement(const aslam::SensorId& sensor_id)
      : Measurement(sensor_id) {}

  explicit Absolute6DoFMeasurement(
      const aslam::SensorId& sensor_id, const int64_t timestamp_nanoseconds,
      const aslam::Transformation& T_G_S,
      const aslam::TransformationCovariance& covariance)
      : Measurement(sensor_id, timestamp_nanoseconds),
        T_G_S_(T_G_S),
        T_G_S_covariance_(covariance) {}

  inline const aslam::Transformation& get_T_G_S() const {
    return T_G_S_;
  }

  inline void set_T_G_S(const aslam::Transformation& T_G_S) {
    T_G_S_ = T_G_S;
  }

  inline const aslam::TransformationCovariance& get_T_G_S_covariance() const {
    return T_G_S_covariance_;
  }

  inline void set_T_G_S_covariance(
      const aslam::TransformationCovariance& measurement_covariance) {
    T_G_S_covariance_ = measurement_covariance;
  }

  inline bool isEqual(
      const Absolute6DoFMeasurement& other, const bool verbose = false) const {
    bool is_equal = true;
    if (other.getSensorId() != getSensorId()) {
      LOG_IF(WARNING, verbose)
          << "Absolute6DoFMeasurements have different sensor id: "
          << other.getSensorId() << " vs " << getSensorId();
      is_equal = false;
    }

    if (other.getTimestampNanoseconds() != getTimestampNanoseconds()) {
      LOG_IF(WARNING, verbose)
          << "Absolute6DoFMeasurements have different timestamps: "
          << other.getTimestampNanoseconds() << " vs "
          << getTimestampNanoseconds();
      is_equal = false;
    }

    constexpr double kPrecision = 1e-6;
    if ((other.get_T_G_S().getTransformationMatrix() -
         get_T_G_S().getTransformationMatrix())
            .cwiseAbs()
            .maxCoeff() > kPrecision) {
      LOG_IF(WARNING, verbose)
          << "Absolute6DoFMeasurements have different T_G_S: "
          << other.get_T_G_S() << " vs " << get_T_G_S();
      is_equal = false;
    }

    if ((other.get_T_G_S_covariance() - get_T_G_S_covariance())
            .cwiseAbs()
            .maxCoeff() > kPrecision) {
      LOG_IF(WARNING, verbose)
          << "Absolute6DoFMeasurements have different T_G_S_covariance: "
          << other.get_T_G_S_covariance() << " vs " << get_T_G_S_covariance();
      is_equal = false;
    }
    return is_equal;
  }

  inline bool has_T_M_B_cached() const {
    return has_T_M_B_cached_;
  }
  inline bool get_T_M_B_cached(aslam::Transformation* T_M_B_cached) const {
    if (has_T_M_B_cached_) {
      *T_M_B_cached = T_M_B_cached_;
      return true;
    }
    return false;
  }
  inline void set_T_M_B_cached(const aslam::Transformation& T_M_B_cached) {
    has_T_M_B_cached_ = true;
    T_M_B_cached_ = T_M_B_cached;
  }

 private:
  inline bool isValidImpl() const {
    return true;
  }

  inline void setRandomImpl() override {
    T_G_S_.setRandom();
    T_G_S_covariance_.setRandom();
  }

  aslam::Transformation T_G_S_;
  aslam::TransformationCovariance T_G_S_covariance_;

  // If the absolute pose constraints arrive in real-time, we can use the
  // pose-lookup buffer to already interpolate the odometry estimate at the time
  // of the measurement and don't need to interpolate it later based on the map
  // state.
  bool has_T_M_B_cached_;
  aslam::Transformation T_M_B_cached_;
};

DEFINE_MEAUREMENT_CONTAINERS(Absolute6DoFMeasurement)

}  // namespace vi_map

DEFINE_MEASUREMENT_HASH(Absolute6DoFMeasurement)

#endif  // SENSORS_ABSOLUTE_6DOF_POSE_H_
