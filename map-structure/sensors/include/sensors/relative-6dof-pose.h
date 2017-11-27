#ifndef SENSORS_RELATIVE_6DOF_POSE_H_
#define SENSORS_RELATIVE_6DOF_POSE_H_

#include <string>

#include <maplab-common/macros.h>
#include <yaml-cpp/yaml.h>

#include "sensors/measurement.h"
#include "sensors/sensor.h"

namespace vi_map {

class Relative6DoFPose final : public Sensor {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  MAPLAB_POINTER_TYPEDEFS(Relative6DoFPose);
  Relative6DoFPose();
  Relative6DoFPose(
      const SensorId& sensor_id, const std::string& hardware_id,
      const aslam::TransformationCovariance& pose_measurement_covariance);
  ~Relative6DoFPose() = default;

  const aslam::TransformationCovariance& getPoseMeasurementCovariance() const {
    return pose_measurement_covariance_;
  }

  Sensor::UniquePtr clone() const override {
    return aligned_unique<Relative6DoFPose>(*this);
  }

 private:
  bool loadFromYamlNodeImpl(const YAML::Node& sensor_node) override;
  void saveToYamlNodeImpl(YAML::Node* sensor_node) const override;

  bool isValidImpl() const override {
    RETURN_FALSE_IF_WRONG_SENSOR_TYPE(kRelative6DoFPose);
    return true;
  }

  void setRandomImpl() override {
    pose_measurement_covariance_.setRandom();
  }

  bool isEqualImpl(const Sensor& other, const double precision) const override {
    const Relative6DoFPose& other_wheel_odometry =
        static_cast<const Relative6DoFPose&>(other);
    return pose_measurement_covariance_.isApprox(
        other_wheel_odometry.pose_measurement_covariance_, precision);
  }

  aslam::TransformationCovariance pose_measurement_covariance_;
};

class Relative6DoFPoseMeasurement : public Measurement {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  MAPLAB_POINTER_TYPEDEFS(Relative6DoFPoseMeasurement);
  Relative6DoFPoseMeasurement() {
    T_R_Sk_.setIdentity();
    measurement_covariance_.setZero();
  }
  Relative6DoFPoseMeasurement(
      const vi_map::SensorId& sensor_id, const int64_t timestamp_nanoseconds,
      const aslam::Transformation& T_R_Sk,
      const aslam::TransformationCovariance& covariance)
      : Measurement(sensor_id, timestamp_nanoseconds),
        T_R_Sk_(T_R_Sk),
        measurement_covariance_(covariance) {}

  const aslam::Transformation& get_T_R_Sk() const {
    return T_R_Sk_;
  }

  void set_T_R_Sk(const aslam::Transformation& T_R_Sk) {
    T_R_Sk_ = T_R_Sk;
  }

  const aslam::TransformationCovariance& getMeasurementCovariance() const {
    CHECK(!measurement_covariance_.isZero());
    return measurement_covariance_;
  }

 private:
  bool isValidImpl() const {
    return true;
  }

  void setRandomImpl() override {
    T_R_Sk_.setRandom();
    measurement_covariance_.setRandom();
  }

  aslam::Transformation T_R_Sk_;
  aslam::TransformationCovariance measurement_covariance_;
};
DEFINE_MEAUREMENT_CONTAINERS(Relative6DoFPoseMeasurement)

}  // namespace vi_map

DEFINE_MEASUREMENT_HASH(Relative6DoFPoseMeasurement)

#endif  // SENSORS_RELATIVE_6DOF_POSE_H_
