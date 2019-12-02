#ifndef SENSORS_GPS_UTM_H_
#define SENSORS_GPS_UTM_H_

#include <string>
#include <unordered_set>
#include <vector>

#include <aslam/common/sensor.h>
#include <maplab-common/macros.h>
#include <yaml-cpp/yaml.h>

#include "sensors/measurement.h"
#include "sensors/sensor-types.h"

namespace vi_map {

class GpsUtm : public aslam::Sensor {
 public:
  MAPLAB_POINTER_TYPEDEFS(GpsUtm);

  GpsUtm();
  explicit GpsUtm(const aslam::SensorId& sensor_id, const std::string& topic);

  Sensor::Ptr cloneAsSensor() const {
    return std::dynamic_pointer_cast<Sensor>(aligned_shared<GpsUtm>(*this));
  }

  uint8_t getSensorType() const override {
    return SensorType::kGpsUtm;
  }

  std::string getSensorTypeString() const override {
    return static_cast<std::string>(kGpsUtmIdentifier);
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

struct UtmZone final {
  explicit UtmZone(
      const unsigned char longitudinal_zone, const char latitudinal_zone)
      : longitudinal_zone_(longitudinal_zone),
        latitudinal_zone_(latitudinal_zone) {
    CHECK(
        longitudinal_zone_ >= static_cast<unsigned char>(1u) &&
        longitudinal_zone_ <= static_cast<unsigned char>(60u))
        << "Invalid longitudinal UTM zone. Allowed are values between and "
        << "including  1 and 60.";
    CHECK(latitudinal_zone_ >= 'A' && latitudinal_zone_ <= 'Z')
        << "Invalid latitudinal UTM zone. Allowed are capital letters from "
        << "A..Z.";
  }

  static UtmZone createInvalid() {
    return UtmZone();
  }

  std::string getUtmZoneAsString() const {
    return std::to_string(longitudinal_zone_) + latitudinal_zone_;
  }

  bool operator==(const UtmZone& other) const {
    return longitudinal_zone_ == other.longitudinal_zone_ &&
           latitudinal_zone_ == other.latitudinal_zone_;
  }

 private:
  UtmZone() : longitudinal_zone_(0u), latitudinal_zone_(0u) {}

  unsigned char longitudinal_zone_;
  char latitudinal_zone_;
};

// GPS measurement following the  UTM (Universal Transverse Mercator)
// standard coordinate system.
class GpsUtmMeasurement final : public Measurement {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  GpsUtmMeasurement() : utm_zone_(UtmZone::createInvalid()) {
    T_UTM_S_.setIdentity();
  }

  explicit GpsUtmMeasurement(
      const aslam::SensorId& sensor_id, const int64_t timestamp_nanoseconds,
      const aslam::Transformation& T_UTM_S, const UtmZone& utm_zone)
      : Measurement(sensor_id, timestamp_nanoseconds),
        T_UTM_S_(T_UTM_S),
        utm_zone_(utm_zone) {
    CHECK(isValid());
  }

  bool operator==(const GpsUtmMeasurement& other) const {
    return Measurement::operator==(other) && T_UTM_S_ == other.T_UTM_S_ &&
           utm_zone_ == other.utm_zone_;
  }

  const aslam::Transformation& get_T_UTM_S() const {
    return T_UTM_S_;
  }

 private:
  explicit GpsUtmMeasurement(const aslam::SensorId& sensor_id)
      : Measurement(sensor_id), utm_zone_(UtmZone::createInvalid()) {
    T_UTM_S_.setIdentity();
  }

  bool isValidImpl() const override {
    return true;
  }

  void setRandomImpl() override {
    T_UTM_S_.setRandom();
  }
  // Coordinate frames used:
  // UTM: UTM reference frame.
  // S: GPS sensor reference frame.
  aslam::Transformation T_UTM_S_;
  UtmZone utm_zone_;
};

DEFINE_MEAUREMENT_CONTAINERS(GpsUtmMeasurement)

}  // namespace vi_map

DEFINE_MEASUREMENT_HASH(GpsUtmMeasurement)

#endif  // SENSORS_GPS_UTM_H_
