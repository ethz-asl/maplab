#ifndef VI_MAP_DEPRECATED_OPTIONAL_SENSOR_EXTRINSICS_H_
#define VI_MAP_DEPRECATED_OPTIONAL_SENSOR_EXTRINSICS_H_

#include <string>
#include <unordered_map>

#include <Eigen/Core>
#include <aslam/common/memory.h>
#include <aslam/common/pose-types.h>
#include <maplab-common/macros.h>
#include <sensors/sensor.h>

#include "vi-map/unique-id.h"
#include "vi-map/vi_map.pb.h"
#include "vi-map/vi_map_deprecated.pb.h"

namespace vi_map {

enum class OptionalSensorType { kWheelOdometry = 1, kGPSUTM = 2, kGPSWGS = 3 };

const char kSensorTypeWheelOdometry[] = "wheel-odometry";
const char kSensorTypeGPSUTM[] = "gps-utm";
const char kSensorTypeGPSWGS[] = "gps-wgs";

std::string convertOptionalSensorTypeToString(OptionalSensorType sensor_type);
OptionalSensorType convertOptionalSensorTypeAsStringToOptionalSensorType(
    const std::string& optional_sensor_type_as_string);

// Class for storing the extrinsics transformation between some optional sensor
// (i.e. not cam/IMU)
// body frame and the IMU body frame.
// Coordinate frames used:
// S: Sensor body frame.
// I: IMU body frame.
class OptionalSensorExtrinsics {
 public:
  OptionalSensorExtrinsics();
  OptionalSensorExtrinsics(
      const SensorId& id, const OptionalSensorType sensor_type,
      const aslam::Transformation& T_S_I,
      const aslam::TransformationCovariance& T_S_I_covariance);

  virtual ~OptionalSensorExtrinsics() = default;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  MAPLAB_POINTER_TYPEDEFS(OptionalSensorExtrinsics);
  MAPLAB_GET_AS_CASTER

  bool operator==(const OptionalSensorExtrinsics& other) const;
  bool operator!=(const OptionalSensorExtrinsics& other) const;

  inline const SensorId& getId() const {
    return id_;
  }

  inline void setId(const SensorId& id) {
    CHECK(id.isValid());
    id_ = id;
  }

  inline const aslam::Transformation& get_T_S_I() const {
    return T_S_I_;
  }

  inline const aslam::TransformationCovariance& get_T_S_I_Covariance() const {
    return T_S_I_covariance_;
  }

  inline void set_T_S_I(const aslam::Transformation& T_S_I) {
    T_S_I_ = T_S_I;
  }

  inline void set_T_S_I_Covariance(
      const aslam::TransformationCovariance& T_S_I_covariance) {
    T_S_I_covariance_ = T_S_I_covariance;
  }

  inline void set_p_S_I(const aslam::Position3D& p_S_I) {
    T_S_I_.getPosition() = p_S_I;
  }

  inline void set_q_S_I(const Eigen::Quaterniond& q_S_I) {
    T_S_I_.getRotation().toImplementation() = q_S_I;
  }

  inline OptionalSensorType getSensorType() const {
    return sensor_type_;
  }

  static OptionalSensorExtrinsics::UniquePtr loadFromYaml(
      const std::string& yaml_file);
  void saveToYaml(const std::string& yaml_file) const;

  void serialize(
      vi_map_deprecated::proto::OptionalSensorExtrinsics* proto) const;
  void deserialize(
      const vi_map_deprecated::proto::OptionalSensorExtrinsics& proto);

 private:
  SensorId id_;
  OptionalSensorType sensor_type_;

  aslam::Transformation T_S_I_;
  aslam::TransformationCovariance T_S_I_covariance_;
};

typedef AlignedUnorderedMap<SensorId, OptionalSensorExtrinsics>
    OptionalSensorExstrinsicsMap;
typedef Aligned<std::vector, OptionalSensorExtrinsics>
    OptionalSensorExtrinsicsList;

}  // namespace vi_map

#endif  // VI_MAP_DEPRECATED_OPTIONAL_SENSOR_EXTRINSICS_H_
