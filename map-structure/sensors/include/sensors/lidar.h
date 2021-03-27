#ifndef SENSORS_LIDAR_H_
#define SENSORS_LIDAR_H_

#include <memory>
#include <string>
#include <vector>

#include <aslam/common/pose-types.h>
#include <aslam/common/sensor.h>
#include <maplab-common/macros.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <resources-common/point-cloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <yaml-cpp/yaml.h>

#include "sensors/measurement.h"
#include "sensors/sensor-types.h"

namespace vi_map {
class VIMap;
class MeasurementsTest_TestAccessorsLidar_Test;

class Lidar final : public aslam::Sensor {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  MAPLAB_POINTER_TYPEDEFS(Lidar);
  Lidar() = default;
  explicit Lidar(const aslam::SensorId& sensor_id);
  Lidar(const aslam::SensorId& sensor_id, const std::string& topic);

  Sensor::Ptr cloneAsSensor() const {
    return std::dynamic_pointer_cast<Sensor>(aligned_shared<Lidar>(*this));
  }

  Lidar* cloneWithNewIds() const {
    Lidar* cloned_lidar = new Lidar();
    *cloned_lidar = *this;
    aslam::SensorId new_id;
    aslam::generateId(&new_id);
    cloned_lidar->setId(new_id);
    return cloned_lidar;
  }

  uint8_t getSensorType() const override {
    return SensorType::kLidar;
  }

  std::string getSensorTypeString() const override {
    return static_cast<std::string>(kLidarIdentifier);
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
};

template <typename PointCloudType>
class LidarMeasurement : public Measurement {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  MAPLAB_POINTER_TYPEDEFS(LidarMeasurement<PointCloudType>);

  LidarMeasurement() = default;
  LidarMeasurement(
      const aslam::SensorId& sensor_id, const int64_t timestamp_nanoseconds,
      const PointCloudType& point_cloud)
      : Measurement(sensor_id, timestamp_nanoseconds),
        point_cloud_(point_cloud) {
    CHECK(isValid());
  }
  LidarMeasurement(
      const aslam::SensorId& sensor_id, const int64_t timestamp_nanoseconds)
      : Measurement(sensor_id, timestamp_nanoseconds) {
    CHECK(isValid());
  }

  ~LidarMeasurement() = default;

  const PointCloudType& getPointCloud() const {
    return point_cloud_;
  }

  PointCloudType* getPointCloudMutable() {
    return &point_cloud_;
  }

  bool operator==(const LidarMeasurement& other) const {
    return Measurement::operator==(other) &&
           point_cloud_ == other.getPointCloud();
  }

 private:
  explicit LidarMeasurement(const aslam::SensorId& sensor_id)
      : Measurement(sensor_id) {}
  bool isValidImpl() const override;

  void setRandomImpl() override;

  PointCloudType point_cloud_;
};

}  // namespace vi_map

// Adds a new Ouster LIDAR point type to PCL.
namespace pcl {
struct OusterPointType {
  PCL_ADD_POINT4D
  int time_offset_us;
  uint16_t reflectivity;
  uint16_t signal;
  uint8_t ring;
  float intensity;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
}  // namespace pcl

// clang-format off
POINT_CLOUD_REGISTER_POINT_STRUCT(
    pcl::OusterPointType,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (int, time_offset_us, time_offset_us)
    (uint16_t, reflectivity, reflectivity)
    (uint16_t, signal, intensity)
    (uint8_t, ring, ring))
// clang-format on

namespace vi_map {

typedef LidarMeasurement<resources::PointCloud> MaplabLidarMeasurement;
typedef LidarMeasurement<pcl::PointCloud<pcl::PointXYZI>> PclLidarMeasurement;
typedef LidarMeasurement<pcl::PointCloud<pcl::OusterPointType>>
    OusterLidarMeasurement;
typedef LidarMeasurement<sensor_msgs::PointCloud2> RosLidarMeasurement;

DEFINE_MEASUREMENT_CONTAINERS(MaplabLidarMeasurement);
DEFINE_MEASUREMENT_CONTAINERS(PclLidarMeasurement);
DEFINE_MEASUREMENT_CONTAINERS(RosLidarMeasurement);
DEFINE_MEASUREMENT_CONTAINERS(OusterLidarMeasurement);

}  // namespace vi_map

DEFINE_MEASUREMENT_HASH(MaplabLidarMeasurement)
DEFINE_MEASUREMENT_HASH(PclLidarMeasurement)
DEFINE_MEASUREMENT_HASH(RosLidarMeasurement)
DEFINE_MEASUREMENT_HASH(OusterLidarMeasurement)

#endif  // SENSORS_LIDAR_H_
