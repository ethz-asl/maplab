#include "vi-map/optional-sensor-data.h"

#include <glog/logging.h>
#include <maplab-common/accessors.h>

namespace vi_map {

OptionalSensorData::OptionalSensorData(const OptionalSensorData& other)
  : sensor_id_to_gps_utm_measurements_(
      other.sensor_id_to_gps_utm_measurements_),
    sensor_id_to_gps_wgs_measurements_(
        other.sensor_id_to_gps_wgs_measurements_) {}

OptionalSensorData::OptionalSensorData(
    const proto::OptionalSensorData& proto_optional_sensor_data) {
  deserialize(proto_optional_sensor_data);
}

template<>
const GpsUtmMeasurementBuffer& OptionalSensorData::getMeasurements(
    const SensorId& sensor_id) const {
  CHECK(sensor_id.isValid());
  return common::getChecked(sensor_id_to_gps_utm_measurements_, sensor_id);
}

template<>
const GpsWgsMeasurementBuffer& OptionalSensorData::getMeasurements(
    const SensorId& sensor_id) const {
  CHECK(sensor_id.isValid());

  return common::getChecked(sensor_id_to_gps_wgs_measurements_, sensor_id);
}

template <>
bool OptionalSensorData::hasMeasurements<GpsWgsMeasurement>(
    const SensorId& sensor_id) const {
  CHECK(sensor_id.isValid());

  SensorIdToMeasurementsMap<GpsWgsMeasurement>::const_iterator
  gps_wgs_measurements_iterator =
      sensor_id_to_gps_wgs_measurements_.find(sensor_id);
  return gps_wgs_measurements_iterator !=
         sensor_id_to_gps_wgs_measurements_.end();
}

template <>
bool OptionalSensorData::hasMeasurements<GpsUtmMeasurement>(
    const SensorId& sensor_id) const {
  CHECK(sensor_id.isValid());

  SensorIdToMeasurementsMap<GpsUtmMeasurement>::const_iterator
      gps_utm_measurements_iterator =
          sensor_id_to_gps_utm_measurements_.find(sensor_id);
  return gps_utm_measurements_iterator !=
         sensor_id_to_gps_utm_measurements_.end();
}

template <>
GpsUtmMeasurementBuffer& OptionalSensorData::getMeasurementsMutable(
    const SensorId& sensor_id) {
  CHECK(sensor_id.isValid());

  return common::getChecked(sensor_id_to_gps_utm_measurements_, sensor_id);
}

template <>
GpsWgsMeasurementBuffer& OptionalSensorData::getMeasurementsMutable(
    const SensorId& sensor_id) {
  CHECK(sensor_id.isValid());

  return common::getChecked(sensor_id_to_gps_wgs_measurements_, sensor_id);
}

template<>
void OptionalSensorData::addMeasurement<GpsUtmMeasurement>(
    const GpsUtmMeasurement& measurement) {
  CHECK_GT(measurement.getTimestampNanoseconds(), 0);
  const vi_map::SensorId& sensor_id = measurement.getSensorId();
  CHECK(sensor_id.isValid());

  std::lock_guard<std::recursive_mutex> lock(mutex_);
  SensorIdToMeasurementsMap<GpsUtmMeasurement>::iterator
  gps_utm_measurements_iterator =
      sensor_id_to_gps_utm_measurements_.find(sensor_id);

  if (gps_utm_measurements_iterator ==
      sensor_id_to_gps_utm_measurements_.end()) {
    MeasurementBuffer<GpsUtmMeasurement> measurement_buffer;
    measurement_buffer.addValue(
        measurement.getTimestampNanoseconds(), measurement);
    sensor_id_to_gps_utm_measurements_.emplace(sensor_id, measurement_buffer);
  } else {
    gps_utm_measurements_iterator->second.addValue(
        measurement.getTimestampNanoseconds(), measurement);
  }
}

template<>
void OptionalSensorData::addMeasurement<GpsWgsMeasurement>(
    const GpsWgsMeasurement& measurement) {
  CHECK_GT(measurement.getTimestampNanoseconds(), 0);
  const vi_map::SensorId& sensor_id = measurement.getSensorId();
  CHECK(sensor_id.isValid());

  std::lock_guard<std::recursive_mutex> lock(mutex_);
  SensorIdToMeasurementsMap<GpsWgsMeasurement>::iterator
  gps_wgs_measurements_iterator =
      sensor_id_to_gps_wgs_measurements_.find(sensor_id);

  if (gps_wgs_measurements_iterator ==
      sensor_id_to_gps_wgs_measurements_.end()) {
    MeasurementBuffer<GpsWgsMeasurement> measurement_buffer;
    measurement_buffer.addValue(
        measurement.getTimestampNanoseconds(), measurement);
    sensor_id_to_gps_wgs_measurements_.emplace(sensor_id, measurement_buffer);
  } else {
    gps_wgs_measurements_iterator->second.addValue(
        measurement.getTimestampNanoseconds(), measurement);
  }
}

void OptionalSensorData::getAllSensorIds(SensorIdSet* sensor_ids) const {
  CHECK_NOTNULL(sensor_ids)->clear();
  for (const SensorIdToMeasurementsMap<GpsUtmMeasurement>::value_type&
      sensor_id_with_measurement : sensor_id_to_gps_utm_measurements_) {
    const SensorId& sensor_id = sensor_id_with_measurement.first;
    CHECK(sensor_id.isValid());
    CHECK(sensor_ids->emplace(sensor_id).second);
  }
  for (const SensorIdToMeasurementsMap<GpsWgsMeasurement>::value_type&
      sensor_id_with_measurement : sensor_id_to_gps_wgs_measurements_) {
    const SensorId& sensor_id = sensor_id_with_measurement.first;
    CHECK(sensor_id.isValid());
    CHECK(sensor_ids->emplace(sensor_id).second);
  }
}

void OptionalSensorData::serialize(
    proto::OptionalSensorData* proto_optional_sensor_data) const {
  CHECK_NOTNULL(proto_optional_sensor_data);
  std::lock_guard<std::recursive_mutex> lock(mutex_);

  for (const SensorIdToMeasurementsMap<GpsUtmMeasurement>::value_type&
      sensor_id_with_measurement : sensor_id_to_gps_utm_measurements_) {
    const SensorId& sensor_id = sensor_id_with_measurement.first;
    CHECK(sensor_id.isValid());

    for (const SensorIdToMeasurementsMap<GpsUtmMeasurement>::
        value_type::second_type::BufferType::value_type&
        gps_utm_measurement_with_timestamp :
        sensor_id_with_measurement.second.buffered_values()) {
      CHECK_EQ(
          gps_utm_measurement_with_timestamp.first,
          gps_utm_measurement_with_timestamp.second.getTimestampNanoseconds());
      gps_utm_measurement_with_timestamp.second.serialize(
          proto_optional_sensor_data->add_gps_utm_measurements());
    }
  }

  for (const SensorIdToMeasurementsMap<GpsWgsMeasurement>::value_type&
      sensor_id_with_measurement : sensor_id_to_gps_wgs_measurements_) {
    const SensorId& sensor_id = sensor_id_with_measurement.first;
    CHECK(sensor_id.isValid());

    for (const SensorIdToMeasurementsMap<GpsWgsMeasurement>::
        value_type::second_type::BufferType::value_type&
        gps_wgs_measurement_with_timestamp :
        sensor_id_with_measurement.second.buffered_values()) {
      CHECK_EQ(
          gps_wgs_measurement_with_timestamp.first,
          gps_wgs_measurement_with_timestamp.second.getTimestampNanoseconds());
      gps_wgs_measurement_with_timestamp.second.serialize(
          proto_optional_sensor_data->add_gps_wgs_measurements());
    }
  }
}

void OptionalSensorData::deserialize(
    const proto::OptionalSensorData& proto_optional_sensor_data) {
  std::lock_guard<std::recursive_mutex> lock(mutex_);

  for (const measurements::proto::GpsUtmMeasurement proto_gps_utm_measurement :
       proto_optional_sensor_data.gps_utm_measurements()) {
    GpsUtmMeasurement gps_utm_measurement;
    gps_utm_measurement.deserialize(proto_gps_utm_measurement);
    addMeasurement(gps_utm_measurement);
  }
  for (const measurements::proto::GpsWgsMeasurement proto_gps_wgs_measurement :
       proto_optional_sensor_data.gps_wgs_measurements()) {
    GpsWgsMeasurement gps_wgs_measurement;
    gps_wgs_measurement.deserialize(proto_gps_wgs_measurement);
    addMeasurement(gps_wgs_measurement);
  }
}

}  // namespace vi_map
