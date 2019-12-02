#ifndef SENSORS_ODOMETRY_6DOF_POSE_H_
#define SENSORS_ODOMETRY_6DOF_POSE_H_

#include <string>

#include <aslam/common/sensor.h>
#include <maplab-common/macros.h>
#include <yaml-cpp/yaml.h>

#include "sensors/measurement.h"
#include "sensors/sensor-types.h"

namespace vi_map {

// Represents a 6DoF Odometry sensor, that provides a transformation from the
// odometry sensor frame to the odometry base frame, which is assumed to be the
// mission frame.
class Odometry6DoF final : public aslam::Sensor {
 public:
  MAPLAB_POINTER_TYPEDEFS(Odometry6DoF);

  Odometry6DoF();
  explicit Odometry6DoF(
      const aslam::SensorId& sensor_id, const std::string& topic);

  Sensor::Ptr cloneAsSensor() const {
    return std::dynamic_pointer_cast<Sensor>(
        aligned_shared<Odometry6DoF>(*this));
  }

  Odometry6DoF* cloneWithNewIds() const {
    Odometry6DoF* cloned_odom_sensor = new Odometry6DoF();
    *cloned_odom_sensor = *this;
    aslam::SensorId new_id;
    aslam::generateId(&new_id);
    cloned_odom_sensor->setId(new_id);
    return cloned_odom_sensor;
  }

  uint8_t getSensorType() const override {
    return SensorType::kOdometry6DoF;
  }

  std::string getSensorTypeString() const override {
    return static_cast<std::string>(kOdometry6DoFIdentifier);
  }

  inline bool has_T_St_Stp1_fixed_covariance() {
    return has_fixed_covariance_;
  }

  inline bool get_T_St_Stp1_fixed_covariance(
      aslam::TransformationCovariance* T_St_Stp1_fixed_covariance) const {
    if (has_fixed_covariance_) {
      *T_St_Stp1_fixed_covariance = T_St_Stp1_fixed_covariance_;
      return true;
    }
    return false;
  }

  inline void set_T_St_Stp1_fixed_covariance(
      const aslam::TransformationCovariance& T_St_Stp1_fixed_covariance) {
    T_St_Stp1_fixed_covariance_ = T_St_Stp1_fixed_covariance;
    has_fixed_covariance_ = true;
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

  bool has_fixed_covariance_;

  aslam::TransformationCovariance T_St_Stp1_fixed_covariance_;
};

// Stores 6DoF Odometry measurements, i.e. a transformation from the odometry
// sensor frame (S) to the odometry base frame (O), which is assumed to be the
// mission frame (M).
class Odometry6DoFMeasurement : public Measurement {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  MAPLAB_POINTER_TYPEDEFS(Odometry6DoFMeasurement);

  Odometry6DoFMeasurement() {
    T_O_S_.setIdentity();
    T_O_S_covariance_.setZero();
  }

  explicit Odometry6DoFMeasurement(
      const aslam::SensorId& sensor_id, const int64_t timestamp_nanoseconds,
      const aslam::Transformation& T_O_S,
      const aslam::TransformationCovariance& covariance)
      : Measurement(sensor_id, timestamp_nanoseconds),
        T_O_S_(T_O_S),
        T_O_S_covariance_(covariance) {}

  const aslam::Transformation& get_T_O_S() const {
    return T_O_S_;
  }

  void set_T_O_S(const aslam::Transformation& T_O_S) {
    T_O_S_ = T_O_S;
  }

  const aslam::TransformationCovariance& get_T_O_S_covariance() const {
    return T_O_S_covariance_;
  }

  void set_T_O_S_covariance(
      const aslam::TransformationCovariance& measurement_covariance) {
    T_O_S_covariance_ = measurement_covariance;
  }

 private:
  bool isValidImpl() const {
    return true;
  }

  void setRandomImpl() override {
    T_O_S_.setRandom();
    T_O_S_covariance_.setRandom();
  }

  aslam::Transformation T_O_S_;
  aslam::TransformationCovariance T_O_S_covariance_;
};

DEFINE_MEAUREMENT_CONTAINERS(Odometry6DoFMeasurement)

constexpr char kYamlFieldNameFixedCovariance[] = "T_St_Stp1_fixed_covariance";

}  // namespace vi_map

DEFINE_MEASUREMENT_HASH(Odometry6DoFMeasurement)

#endif  // SENSORS_ODOMETRY_6DOF_POSE_H_
