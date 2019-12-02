#ifndef SENSORS_GPS_WGS_H_
#define SENSORS_GPS_WGS_H_

#include <string>
#include <unordered_set>
#include <vector>

#include <aslam/common/sensor.h>
#include <maplab-common/macros.h>
#include <yaml-cpp/yaml.h>

#include "sensors/measurement.h"
#include "sensors/sensor-types.h"

namespace vi_map {

class GpsWgs : public aslam::Sensor {
 public:
  MAPLAB_POINTER_TYPEDEFS(GpsWgs);

  GpsWgs();
  explicit GpsWgs(const aslam::SensorId& sensor_id, const std::string& topic);

  uint8_t getSensorType() const override {
    return SensorType::kGpsWgs;
  }

  std::string getSensorTypeString() const override {
    return static_cast<std::string>(kGpsWgsIdentifier);
  }

  Sensor::Ptr cloneAsSensor() const {
    return std::dynamic_pointer_cast<Sensor>(aligned_shared<GpsWgs>(*this));
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

// GPS measurement following the WGS (World Geodetic System) 84 standard
// coordinate system.
class GpsWgsMeasurement final : public Measurement {
 public:
  GpsWgsMeasurement()
      : latitude_deg_(0.0), longitude_deg_(0.0), altitude_meters_(0.0) {}
  explicit GpsWgsMeasurement(
      const aslam::SensorId& sensor_id, const int64_t timestamp_nanoseconds,
      const double latitude_deg, const double longitude_deg,
      const double altitude_meters)
      : Measurement(sensor_id, timestamp_nanoseconds),
        latitude_deg_(latitude_deg),
        longitude_deg_(longitude_deg),
        altitude_meters_(altitude_meters) {
    CHECK(isValid());
  }

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

  static constexpr double kMinLatitudeDeg = -90.0;
  static constexpr double kMaxLatitudeDeg = 90.0;
  static constexpr double kMinLongitudeDeg = -180.0;
  static constexpr double kMaxLongitudeDeg = 180.0;
  static constexpr double kMinAltitudeMeters = -1e3;
  static constexpr double kMaxAltitudeMeters = 2e5;

 private:
  explicit GpsWgsMeasurement(const aslam::SensorId& sensor_id)
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
