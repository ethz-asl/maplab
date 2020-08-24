#ifndef SENSORS_EXTERNAL_FEATURES_H_
#define SENSORS_EXTERNAL_FEATURES_H_

#include <string>

#include <aslam/common/sensor.h>
#include <maplab-common/macros.h>
#include <yaml-cpp/yaml.h>

#include "sensors/measurement.h"
#include "sensors/sensor-types.h"

namespace vi_map {

enum ExternalFeatureType : uint8_t {
  kVisualBinaryFeatures,
  kVisualFloatFeatures
};

// ...
class ExternalFeatures final : public aslam::Sensor {
 public:
  MAPLAB_POINTER_TYPEDEFS(ExternalFeatures);

  ExternalFeatures();
  explicit ExternalFeatures(
      const aslam::SensorId& sensor_id, const std::string& topic);

  Sensor::Ptr cloneAsSensor() const {
    return std::dynamic_pointer_cast<Sensor>(
        aligned_shared<ExternalFeatures>(*this));
  }

  ExternalFeatures* cloneWithNewIds() const {
    ExternalFeatures* cloned_external_features = new ExternalFeatures();
    *cloned_external_features = *this;
    aslam::SensorId new_id;
    aslam::generateId(&new_id);
    cloned_external_features->setId(new_id);
    return cloned_external_features;
  }

  uint8_t getSensorType() const override {
    return SensorType::kExternalFeatures;
  }

  std::string getSensorTypeString() const override {
    return static_cast<std::string>(kExternalFeaturesIdentifier);
  }

  const aslam::SensorId& getTargetSensorId() const {
    return target_sensor_id_;
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

  aslam::SensorId target_sensor_id_;
  bool has_uncertainties_;
  bool has_orientations_;
  bool has_scores_;
  bool has_scales_;
  bool has_track_ids;
  ExternalFeatureType feature_type_;
};

// ...
class ExternalFeaturesMeasurement : public Measurement {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  MAPLAB_POINTER_TYPEDEFS(ExternalFeaturesMeasurement);

  ExternalFeaturesMeasurement() {}

  explicit ExternalFeaturesMeasurement(const aslam::SensorId& sensor_id)
      : Measurement(sensor_id) {}

  explicit ExternalFeaturesMeasurement(
      const aslam::SensorId& sensor_id, const int64_t timestamp_nanoseconds)
      : Measurement(sensor_id, timestamp_nanoseconds) {}

  inline bool isEqual(
      const ExternalFeaturesMeasurement& other,
      const bool verbose = false) const {
    bool is_equal = true;
    if (other.getSensorId() != getSensorId()) {
      LOG_IF(WARNING, verbose)
          << "ExternalFeaturesMeasurements have different sensor id: "
          << other.getSensorId() << " vs " << getSensorId();
      is_equal = false;
    }

    if (other.getTimestampNanoseconds() != getTimestampNanoseconds()) {
      LOG_IF(WARNING, verbose)
          << "ExternalFeaturesMeasurements have different timestamps: "
          << other.getTimestampNanoseconds() << " vs "
          << getTimestampNanoseconds();
      is_equal = false;
    }

    return is_equal;
  }

 private:
  inline bool isValidImpl() const {
    return true;
  }

  inline void setRandomImpl() override {}
};

DEFINE_MEAUREMENT_CONTAINERS(ExternalFeaturesMeasurement)

}  // namespace vi_map

DEFINE_MEASUREMENT_HASH(ExternalFeaturesMeasurement)

#endif  // SENSORS_EXTERNAL_FEATURES_H_
