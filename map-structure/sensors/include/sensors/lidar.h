#ifndef SENSORS_LIDAR_H_
#define SENSORS_LIDAR_H_

#include <memory>
#include <string>
#include <vector>

#include <aslam/common/pose-types.h>
#include <maplab-common/macros.h>
#include <resources-common/point-cloud.h>
#include <yaml-cpp/yaml.h>

#include "sensors/measurement.h"
#include "sensors/sensor.h"

namespace vi_map {
class VIMap;
class MeasurementsTest_TestAccessorsLidar_Test;
namespace test {
void generateOptionalSensorDataAndAddToMap(VIMap* map);
}

class Lidar : public Sensor {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  MAPLAB_POINTER_TYPEDEFS(Lidar);
  Lidar();
  Lidar(const SensorId& sensor_id, const std::string& hardware_id);

  Sensor::UniquePtr clone() const override {
    return aligned_unique<Lidar>(*this);
  }

 private:
  bool loadFromYamlNodeImpl(const YAML::Node& sensor_node) override;
  void saveToYamlNodeImpl(YAML::Node* sensor_node) const override;

  bool isValidImpl() const override {
    RETURN_FALSE_IF_WRONG_SENSOR_TYPE(kLidar);
    return true;
  }

  bool isEqualImpl(
      const Sensor& /*other*/, const double /*precision*/) const override {
    RETURN_FALSE_IF_WRONG_SENSOR_TYPE(kLidar);
    return true;
  }

  void setRandomImpl() override {}
};

class LidarMeasurement final : public Measurement {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  LidarMeasurement() = default;
  LidarMeasurement(
      const SensorId& sensor_id, const int64_t timestamp_nanoseconds)
      : Measurement(sensor_id, timestamp_nanoseconds) {
    CHECK(isValid());
  }
  ~LidarMeasurement() = default;

  const resources::PointCloud& getPointCloud() const {
    return point_cloud_;
  }

  resources::PointCloud* getPointCloudMutable() {
    return &point_cloud_;
  }

  bool operator==(const LidarMeasurement& other) const {
    return Measurement::operator==(other) &&
           point_cloud_ == other.getPointCloud();
  }

  static constexpr double kMinDistanceMeters = 0.0;
  static constexpr double kMaxDistanceMeters = 1e3;
  static constexpr double kMinIntensity = 0.0;
  static constexpr double kMaxIntensity = 255.0;

 private:
  explicit LidarMeasurement(const vi_map::SensorId& sensor_id)
      : vi_map::Measurement(sensor_id) {}
  bool isValidImpl() const override;

  void setRandomImpl() override;

  resources::PointCloud point_cloud_;
};

DEFINE_MEAUREMENT_CONTAINERS(LidarMeasurement);

}  // namespace vi_map

DEFINE_MEASUREMENT_HASH(LidarMeasurement)

#endif  // SENSORS_LIDAR_H_
