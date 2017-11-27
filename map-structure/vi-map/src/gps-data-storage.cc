#include "vi-map/deprecated/gps-data-storage.h"

#include <maplab-common/eigen-proto.h>

namespace vi_map {

void GPSDataStorage::swap(GPSDataStorage* other) {
  wgs_data_.swap(other->wgs_data_);
  utm_data_.swap(other->utm_data_);
}

void GPSDataStorage::importGPSData(const GPSDataStorage& other) {
  for (const UTMContainer::value_type& mission_id_utm_data_pair :
       other.getUTMContainer()) {
    const MissionId& mission_id = mission_id_utm_data_pair.first;
    CHECK(mission_id.isValid());
    if (utm_data_.count(mission_id) > 0u) {
      LOG(WARNING) << "Skipping copy of UTM data of mission "
                   << mission_id.hexString()
                   << " as it would overwrite already present data.";
      continue;
    }
    CHECK(
        utm_data_.emplace(mission_id, mission_id_utm_data_pair.second).second);
  }
  for (const WGSContainer::value_type& mission_id_wgs_data_pair :
       other.getWGSContainer()) {
    const MissionId& mission_id = mission_id_wgs_data_pair.first;
    CHECK(mission_id.isValid());
    if (wgs_data_.count(mission_id) > 0u) {
      LOG(WARNING) << "Skipping copy of WGS data of mission "
                   << mission_id.hexString()
                   << " as it would overwrite already present data.";
      continue;
    }
    CHECK(
        wgs_data_.emplace(mission_id, mission_id_wgs_data_pair.second).second);
  }
}

size_t GPSDataStorage::getNumWGSMeasurements() const {
  size_t num_measurements = 0u;
  for (const WGSContainer::value_type& mission_buffer_pair : wgs_data_) {
    num_measurements += mission_buffer_pair.second.size();
  }
  return num_measurements;
}

size_t GPSDataStorage::getNumUTMMeasurements() const {
  size_t num_measurements = 0u;
  for (const UTMContainer::value_type& mission_buffer_pair : utm_data_) {
    num_measurements += mission_buffer_pair.second.size();
  }
  return num_measurements;
}

void GPSDataStorage::duplicateGpsDataOfMission(
    const MissionId& source_mission_id,
    const MissionId& destination_mission_id) {
  CHECK(source_mission_id.isValid());
  CHECK(destination_mission_id.isValid());
  if (hasWGSBufferForMission(source_mission_id)) {
    CHECK(
        wgs_data_
            .emplace(
                destination_mission_id,
                getWGSBufferForMission(source_mission_id))
            .second);
  }
  if (hasUTMBufferForMission(source_mission_id)) {
    CHECK(
        utm_data_
            .emplace(
                destination_mission_id,
                getUTMBufferForMission(source_mission_id))
            .second);
  }
}

void GPSDataStorage::serialize(
    vi_map_deprecated::proto::GPSDataStorage* proto_gps_data_storage) const {
  CHECK_NOTNULL(proto_gps_data_storage);

  size_t num_serialized_wgs_measurements = 0u;
  size_t num_serialized_wgs_buffers = 0u;
  for (const WGSContainer::value_type& mission_buffer_pair : wgs_data_) {
    vi_map_deprecated::proto::MissionIdWithGPSWGSData*
    proto_mission_id_with_wgs_data =
        CHECK_NOTNULL(
            proto_gps_data_storage->add_mission_ids_with_gps_wgs_data());

    const MissionId& mission_id = mission_buffer_pair.first;
    CHECK(mission_id.isValid());
    mission_id.serialize(proto_mission_id_with_wgs_data->mutable_mission_id());

    const GPSBuffer<GPSMeasurementWGS>& gps_wgs_buffer =
        mission_buffer_pair.second;
    gps_wgs_buffer.lockContainer();
    for (const GPSBuffer<GPSMeasurementWGS>::BufferType::value_type&
             time_gps_pair : gps_wgs_buffer.buffered_values()) {
      const GPSMeasurementWGS& wgs_measurement = time_gps_pair.second;

      vi_map_deprecated::proto::GPSMeasurementWGS* proto_wgs_measurement =
          proto_mission_id_with_wgs_data->add_wgs_measurements();
      CHECK_NOTNULL(proto_wgs_measurement);

      CHECK(wgs_measurement.sensor_id.isValid());
      wgs_measurement.sensor_id.serialize(
          proto_wgs_measurement->mutable_sensor_id());
      proto_wgs_measurement->set_timestamp_nanoseconds(
          wgs_measurement.timestamp);
      proto_wgs_measurement->set_latitude_deg(wgs_measurement.latitude_deg);
      proto_wgs_measurement->set_longitude_deg(wgs_measurement.longitude_deg);
      proto_wgs_measurement->set_altitude_meters(
          wgs_measurement.altitude_meters);
      ++num_serialized_wgs_measurements;
    }
    gps_wgs_buffer.unlockContainer();
    ++num_serialized_wgs_buffers;
  }
  VLOG(3) << "Serialized " << num_serialized_wgs_measurements
          << " GPS measurements in WGS format.";

  size_t num_serialized_utm_measurements = 0u;
  size_t num_serialized_utm_buffers = 0u;
  for (const UTMContainer::value_type& mission_buffer_pair : utm_data_) {
    vi_map_deprecated::proto::MissionIdWithGPSUTMData*
        proto_mission_id_with_utm_data = CHECK_NOTNULL(
            proto_gps_data_storage->add_mission_ids_with_gps_utm_data());

    MissionId mission_id = mission_buffer_pair.first;
    CHECK(mission_id.isValid());
    mission_id.serialize(proto_mission_id_with_utm_data->mutable_mission_id());

    const GPSBuffer<GPSMeasurementUTM>& gps_utm_buffer =
        mission_buffer_pair.second;
    gps_utm_buffer.lockContainer();
    for (const GPSBuffer<GPSMeasurementUTM>::BufferType::value_type&
             time_gps_pair : gps_utm_buffer.buffered_values()) {
      const GPSMeasurementUTM& utm_measurement = time_gps_pair.second;

      vi_map_deprecated::proto::GPSMeasurementUTM* proto_utm_measurement =
          proto_mission_id_with_utm_data->add_utm_measurements();
      CHECK_NOTNULL(proto_utm_measurement);

      CHECK(utm_measurement.sensor_id.isValid());
      utm_measurement.sensor_id.serialize(
          proto_utm_measurement->mutable_sensor_id());
      proto_utm_measurement->set_timestamp_nanoseconds(
          time_gps_pair.second.timestamp);
      common::eigen_proto::serialize(time_gps_pair.second.T_R_S,
                                     proto_utm_measurement->mutable_t_r_s());
      ++num_serialized_utm_measurements;
    }
    gps_utm_buffer.unlockContainer();
    ++num_serialized_utm_buffers;
  }
  VLOG(3) << "Serialized " << num_serialized_utm_measurements
          << " GPS measurements in UTM format.";
}

void GPSDataStorage::deserialize(
    const vi_map_deprecated::proto::GPSDataStorage& proto_gps_data_storage) {
  for (const vi_map_deprecated::proto::MissionIdWithGPSWGSData&
           proto_mission_id_with_wgs_data :
       proto_gps_data_storage.mission_ids_with_gps_wgs_data()) {
    MissionId mission_id;
    mission_id.deserialize(proto_mission_id_with_wgs_data.mission_id());
    CHECK(mission_id.isValid());
    for (const vi_map_deprecated::proto::GPSMeasurementWGS&
             proto_wgs_measurement :
         proto_mission_id_with_wgs_data.wgs_measurements()) {
      GPSMeasurementWGS wgs_measurement;
      wgs_measurement.sensor_id.deserialize(proto_wgs_measurement.sensor_id());
      CHECK(wgs_measurement.sensor_id.isValid());
      wgs_measurement.timestamp = proto_wgs_measurement.timestamp_nanoseconds();
      wgs_measurement.latitude_deg = proto_wgs_measurement.latitude_deg();
      wgs_measurement.longitude_deg = proto_wgs_measurement.longitude_deg();
      wgs_measurement.altitude_meters = proto_wgs_measurement.altitude_meters();
      addMeasurement(wgs_measurement, mission_id);
    }
  }
  VLOG(3) << "Deserialized " << getNumWGSMeasurements()
          << " GPS measurements in WGS format.";

  for (const vi_map_deprecated::proto::MissionIdWithGPSUTMData&
           proto_mission_id_with_utm_data :
       proto_gps_data_storage.mission_ids_with_gps_utm_data()) {
    MissionId mission_id;
    mission_id.deserialize(proto_mission_id_with_utm_data.mission_id());
    CHECK(mission_id.isValid());
    for (const vi_map_deprecated::proto::GPSMeasurementUTM&
             proto_utm_measurement :
         proto_mission_id_with_utm_data.utm_measurements()) {
      GPSMeasurementUTM utm_measurement;
      utm_measurement.sensor_id.deserialize(proto_utm_measurement.sensor_id());
      CHECK(utm_measurement.sensor_id.isValid());
      utm_measurement.timestamp = proto_utm_measurement.timestamp_nanoseconds();
      common::eigen_proto::deserialize(proto_utm_measurement.t_r_s(),
                                       &utm_measurement.T_R_S);
      addMeasurement(utm_measurement, mission_id);
    }
  }

  VLOG(3) << "Deserialized " << getNumUTMMeasurements()
          << " GPS measurements in UTM format.";
}

}  // namespace vi_map
