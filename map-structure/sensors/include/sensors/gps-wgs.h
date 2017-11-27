#ifndef SENSORS_GPS_WGS_H_
#define SENSORS_GPS_WGS_H_

#include <string>
#include <unordered_set>
#include <vector>

#include <aslam/common/pose-types.h>
#include <maplab-common/macros.h>
#include <maplab-common/temporal-buffer.h>
#include <yaml-cpp/yaml.h>

#include "sensors/measurement.h"
#include "sensors/measurements.pb.h"
#include "sensors/sensor.h"

namespace vi_map {
class VIMap;
class MeasurementsTest_TestAccessorsGpsWgs_Test;
namespace test {
void generateOptionalSensorDataAndAddToMap(VIMap* map);
}

class GpsWgs : public Sensor {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  MAPLAB_POINTER_TYPEDEFS(GpsWgs);
  GpsWgs();
  GpsWgs(const SensorId& sensor_id, const std::string& hardware_id);

  Sensor::UniquePtr clone() const override {
    return aligned_unique<GpsWgs>(*this);
  }

 private:
  bool loadFromYamlNodeImpl(const YAML::Node& sensor_node) override;
  void saveToYamlNodeImpl(YAML::Node* sensor_node) const override;

  bool isValidImpl() const override {
    RETURN_FALSE_IF_WRONG_SENSOR_TYPE(kGpsWgs);
    return true;
  }

  bool isEqualImpl(
      const Sensor& /*other*/, const double /*precision*/) const override {
    RETURN_FALSE_IF_WRONG_SENSOR_TYPE(kGpsWgs);
    return true;
  }

  void setRandomImpl() override {}
};

// GPS measurement following the WGS (World Geodetic System) 84 standard
// coordinate system.
class GpsWgsMeasurement final : public Measurement {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  friend class MeasurementsTest_GpsWgsProtoSerialization_Test;
  friend class MeasurementsTest_TestAccessorsGpsWgs_Test;
  friend void test::generateOptionalSensorDataAndAddToMap(VIMap* map);

  GpsWgsMeasurement()
      : latitude_deg_(0.0), longitude_deg_(0.0), altitude_meters_(0.0) {}
  GpsWgsMeasurement(
      const SensorId& sensor_id, const int64_t timestamp_nanoseconds,
      const double latitude_deg, const double longitude_deg,
      const double altitude_meters)
      : Measurement(sensor_id, timestamp_nanoseconds),
        latitude_deg_(latitude_deg),
        longitude_deg_(longitude_deg),
        altitude_meters_(altitude_meters) {
    CHECK(isValid());
  }
  ~GpsWgsMeasurement() = default;

  bool operator==(const GpsWgsMeasurement& other) const {
    return Measurement::operator==(other) &&
           latitude_deg_ == other.getLatitudeDeg() &&
           longitude_deg_ == other.getLongitudeDeg() &&
           altitude_meters_ == other.getAltitudeMeters();
  }

  double getLatitudeDeg() const {
    return latitude_deg_;
  }

  double getLongitudeDeg() const {
    return longitude_deg_;
  }

  double getAltitudeMeters() const {
    return altitude_meters_;
  }

  void serialize(
      measurements::proto::GpsWgsMeasurement* proto_measurement) const;
  void deserialize(
      const measurements::proto::GpsWgsMeasurement& proto_measurement);

  static constexpr double kMinLatitudeDeg = -90.0;
  static constexpr double kMaxLatitudeDeg = 90.0;
  static constexpr double kMinLongitudeDeg = -180.0;
  static constexpr double kMaxLongitudeDeg = 180.0;
  static constexpr double kMinAltitudeMeters = -1e3;
  static constexpr double kMaxAltitudeMeters = 2e5;

 private:
  explicit GpsWgsMeasurement(const SensorId& sensor_id)
      : Measurement(sensor_id),
        latitude_deg_(0.0),
        longitude_deg_(0.0),
        altitude_meters_(0.0) {}
  bool isValidImpl() const override {
    if (latitude_deg_ > kMaxLatitudeDeg || latitude_deg_ < kMinLatitudeDeg) {
      LOG(ERROR) << "Invalid latitude (deg): " << latitude_deg_;
      return false;
    }
    if (longitude_deg_ > kMaxLongitudeDeg ||
        longitude_deg_ < kMinLongitudeDeg) {
      LOG(ERROR) << "Invalid longitude (deg): " << longitude_deg_;
      return false;
    }
    if (altitude_meters_ > kMaxAltitudeMeters ||
        altitude_meters_ < kMinAltitudeMeters) {
      LOG(ERROR) << "Invalid altitude (m): " << altitude_meters_;
      return false;
    }
    return true;
  }

  void setRandomImpl() override;

  double latitude_deg_;
  double longitude_deg_;
  double altitude_meters_;
};
DEFINE_MEAUREMENT_CONTAINERS(GpsWgsMeasurement)

}  // namespace vi_map

DEFINE_MEASUREMENT_HASH(GpsWgsMeasurement)

#endif  // SENSORS_GPS_WGS_H_
