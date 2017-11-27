#ifndef SENSORS_SENSOR_H_
#define SENSORS_SENSOR_H_

#include <string>

#include <Eigen/Core>
#include <aslam/common/memory.h>
#include <aslam/common/pose-types.h>
#include <aslam/common/yaml-serialization.h>
#include <glog/logging.h>
#include <maplab-common/macros.h>
#include <maplab-common/string-tools.h>
#include <maplab-common/unique-id.h>
#include <maplab-common/yaml-file-serializable.h>

namespace vi_map {

#define RETURN_FALSE_IF_WRONG_SENSOR_TYPE(sensor_type)                       \
  if (getSensorType() != SensorType::sensor_type) {                          \
    LOG(ERROR) << "Mismatching sensor type. Should be "                      \
               << sensorTypeToString(SensorType::sensor_type) << ", but is " \
               << sensorTypeToString(getSensorType()) << '.';                \
    return false;                                                            \
  }

UNIQUE_ID_DEFINE_ID(SensorId);

enum class SensorType : int {
  kImu,
  kRelative6DoFPose,
  kGpsUtm,
  kGpsWgs,
  kInvalidSensor
};

template <class Sensor>
SensorType sensorToType();

constexpr char kSensorTypeImu[] = "IMU";
constexpr char kSensorTypeRelative6DoFPose[] = "Relative_6DoF_Pose";
constexpr char kSensorTypeGpsUtm[] = "GPS_UTM";
constexpr char kSensorTypeGpsWgs[] = "GPS_WGS";

inline std::string sensorTypeToString(const SensorType sensor_type) {
  switch (sensor_type) {
    case SensorType::kImu:
      return static_cast<std::string>(kSensorTypeImu);
      break;
    case SensorType::kRelative6DoFPose:
      return static_cast<std::string>(kSensorTypeRelative6DoFPose);
      break;
    case SensorType::kGpsUtm:
      return static_cast<std::string>(kSensorTypeGpsUtm);
      break;
    case SensorType::kGpsWgs:
      return static_cast<std::string>(kSensorTypeGpsWgs);
      break;
    default:
      LOG(FATAL) << "Unknown sensor type: " << static_cast<int>(sensor_type);
      break;
  }
  return "";
}

inline SensorType stringToSensorType(const std::string& sensor_type) {
  CHECK(!sensor_type.empty());
  if (sensor_type == static_cast<std::string>(kSensorTypeRelative6DoFPose)) {
    return SensorType::kRelative6DoFPose;
  } else if (sensor_type == static_cast<std::string>(kSensorTypeImu)) {
    return SensorType::kImu;
  } else if (sensor_type == static_cast<std::string>(kSensorTypeGpsUtm)) {
    return SensorType::kGpsUtm;
  } else if (sensor_type == static_cast<std::string>(kSensorTypeGpsWgs)) {
    return SensorType::kGpsWgs;
  } else {
    LOG(FATAL) << "Unknown sensor type: " << sensor_type;
    return SensorType::kInvalidSensor;
  }
}

class Sensor : public common::YamlFileSerializable {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  MAPLAB_POINTER_TYPEDEFS(Sensor);
  friend class SensorManagerTest;
  template <class DerivedSensor>
  friend typename DerivedSensor::UniquePtr createTestSensor();

  Sensor(SensorType sensor_type, const std::string& hardware_id);
  Sensor(
      const SensorId& sensor_id, SensorType sensor_type,
      const std::string& hardware_id);

  virtual ~Sensor() = default;

  virtual Sensor::UniquePtr clone() const = 0;

  const SensorId& getId() const {
    return id_;
  }

  void setId(const SensorId& sensor_id) {
    id_ = sensor_id;
    CHECK(id_.isValid());
  }

  SensorType getSensorType() const {
    return sensor_type_;
  }

  const std::string& getHardwareId() const {
    return hardware_id_;
  }

  bool operator==(const Sensor& other) const {
    constexpr double kPrecision = 1e-12;
    return isEqual(other, kPrecision);
  }
  // void saveToYaml(const std::string& yaml_path) const;
  void serialize(YAML::Node* yaml_node) const override;
  bool deserialize(const YAML::Node& sensor_node) override;

  bool isValid() const {
    if (!id_.isValid()) {
      LOG(ERROR) << "Invalid sensor id.";
      return false;
    }
    if (sensor_type_ == SensorType::kInvalidSensor) {
      LOG(ERROR) << "Invalid sensor type.";
      return false;
    }
    return isValidImpl();
  }

 protected:
  // This constructor is intended only for deserialization after construction.
  Sensor();
  explicit Sensor(SensorType sensor_type);

 private:
  void setRandom() {
    common::generateId(&id_);
    constexpr size_t kHardwareIdLength = 10u;
    hardware_id_ = common::createRandomString(kHardwareIdLength);
    setRandomImpl();
  }

  bool isEqual(const Sensor& other, const double precision) const {
    return id_ == other.id_ && sensor_type_ == other.sensor_type_ &&
           hardware_id_ == other.hardware_id_ && isEqualImpl(other, precision);
  }

  virtual bool loadFromYamlNodeImpl(const YAML::Node& sensor_node) = 0;
  virtual void saveToYamlNodeImpl(YAML::Node* sensor_node) const = 0;

  virtual bool isValidImpl() const = 0;

  virtual void setRandomImpl() = 0;

  virtual bool isEqualImpl(
      const Sensor& other, const double precision) const = 0;

  SensorId id_;
  SensorType sensor_type_;
  std::string hardware_id_;
};
typedef AlignedUnorderedMap<SensorId, Sensor::UniquePtr> SensorIdToSensorMap;

}  // namespace vi_map

UNIQUE_ID_DEFINE_ID_HASH(vi_map::SensorId);

#endif  // SENSORS_SENSOR_H_
