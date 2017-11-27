#ifndef SENSORS_SENSOR_SYSTEM_H_
#define SENSORS_SENSOR_SYSTEM_H_

#include <string>

#include <Eigen/Core>
#include <aslam/common/memory.h>
#include <aslam/common/pose-types.h>
#include <maplab-common/macros.h>
#include <maplab-common/unique-id.h>
#include <maplab-common/yaml-file-serializable.h>
#include <yaml-cpp/yaml.h>

#include "sensors/sensor-extrinsics.h"
#include "sensors/sensor.h"

namespace vi_map {

UNIQUE_ID_DEFINE_ID(SensorSystemId);

class SensorSystem final : common::YamlFileSerializable {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  MAPLAB_POINTER_TYPEDEFS(SensorSystem);
  explicit SensorSystem(const SensorId& reference_sensor_id);
  static SensorSystem::UniquePtr createFromYaml(
      const std::string& yaml_filepath);
  static SensorSystem::UniquePtr createFromYaml(const YAML::Node& yaml_node);
  ~SensorSystem() = default;

  void setSensorExtrinsics(
      const SensorId& sensor_id, const Extrinsics& extrinsics);
  const aslam::Transformation& getSensor_T_R_S(const SensorId& sensor_id) const;
  const aslam::Position3D& getSensor_p_R_S(const SensorId& sensor_id) const;

  ExtrinsicsType getSensorExtrinsicsType(const SensorId& sensor_id) const;

  void getAllSensorIds(SensorIdSet* sensor_ids) const;
  bool hasSensor(const SensorId& sensor_id) const;
  bool hasExtrinsicsForSensor(const SensorId& sensor_id) const {
    return hasSensor(sensor_id) && reference_sensor_id_ != sensor_id;
  }

  const SensorSystemId& getId() const {
    CHECK(id_.isValid());
    return id_;
  }

  void serialize(YAML::Node* yaml_node) const override;

  bool operator==(const SensorSystem& other) const {
    return id_ == other.id_ &&
           reference_sensor_id_ == other.reference_sensor_id_ &&
           sensor_id_to_extrinsics_map_ == other.sensor_id_to_extrinsics_map_;
  }

 private:
  SensorSystem();

  const Extrinsics& getSensorExtrinsics(const SensorId& sensor_id) const;

  bool deserialize(const YAML::Node& yaml_node) override;

  SensorSystemId id_;
  SensorId reference_sensor_id_;

  typedef AlignedUnorderedMap<SensorId, Extrinsics> ExtrinsicsMap;
  ExtrinsicsMap sensor_id_to_extrinsics_map_;
};

}  // namespace vi_map

UNIQUE_ID_DEFINE_ID_HASH(vi_map::SensorSystemId);

#endif  // SENSORS_SENSOR_SYSTEM_H_
