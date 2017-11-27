#ifndef VI_MAP_DEPRECATED_GPS_DATA_STORAGE_INL_H_
#define VI_MAP_DEPRECATED_GPS_DATA_STORAGE_INL_H_

namespace vi_map {

template <>
inline void GPSDataStorage::addMeasurement(
    const GPSMeasurementWGS& measurement, const MissionId& mission_id) {
  CHECK_GT(measurement.timestamp, 0);
  CHECK(measurement.sensor_id.isValid());
  CHECK(mission_id.isValid());
  wgs_data_[mission_id].addValue(measurement.timestamp, measurement);
}

template <>
inline void GPSDataStorage::addMeasurement(
    const GPSMeasurementUTM& measurement, const MissionId& mission_id) {
  CHECK_GT(measurement.timestamp, 0);
  CHECK(measurement.sensor_id.isValid());
  CHECK(mission_id.isValid());
  utm_data_[mission_id].addValue(measurement.timestamp, measurement);
}

inline void GPSDataStorage::clearWGSData(void) {
  wgs_data_.clear();
}

inline void GPSDataStorage::clearUTMData(void) {
  utm_data_.clear();
}

template <>
inline bool GPSDataStorage::getClosestGPSMeasurementInTimeForMission(
    const MissionId& mission_id, int64_t timestamp,
    GPSMeasurementUTM* gps_measurement) const {
  CHECK_NOTNULL(gps_measurement);
  if (hasUTMData()) {
    UTMContainer::const_iterator utm_container_iter =
        utm_data_.find(mission_id);
    if (utm_container_iter != utm_data_.end()) {
      return utm_container_iter->second.getNearestValueToTime(
          timestamp, gps_measurement);
    } else {
      LOG(WARNING) << "Could not retrieve GPS (UTM) measurement for provided "
                      "mission ID "
                   << mission_id << " and timestamp " << timestamp;
      return false;
    }
  } else {
    LOG(WARNING) << "No GPS data in UTM format available.";
    return false;
  }
}

template <>
inline bool GPSDataStorage::getClosestGPSMeasurementInTimeForMission(
    const MissionId& mission_id, int64_t timestamp,
    GPSMeasurementWGS* gps_measurement) const {
  CHECK_NOTNULL(gps_measurement);
  if (hasWGSData()) {
    WGSContainer::const_iterator wgs_container_iter =
        wgs_data_.find(mission_id);
    if (wgs_container_iter != wgs_data_.end()) {
      return wgs_container_iter->second.getNearestValueToTime(
          timestamp, gps_measurement);
    } else {
      LOG(WARNING) << "Could not retrieve GPS (WGS) measurement for provided "
                   << "mission ID " << mission_id << " and timestamp "
                   << timestamp << ".";
      return false;
    }
  } else {
    LOG(WARNING) << "No GPS data in WGS format available.";
    return false;
  }
}

inline bool GPSDataStorage::hasWGSData() const {
  return !wgs_data_.empty();
}

inline bool GPSDataStorage::hasUTMData() const {
  return !utm_data_.empty();
}

template <>
inline bool GPSDataStorage::hasData<GPSMeasurementUTM>() const {
  return hasUTMData();
}

template <>
inline bool GPSDataStorage::hasData<GPSMeasurementWGS>() const {
  return hasWGSData();
}

inline size_t GPSDataStorage::getNumMissionsWithWGSMeasurements() const {
  return wgs_data_.size();
}

inline size_t GPSDataStorage::getNumMissionsWithUTMMeasurements() const {
  return utm_data_.size();
}

inline const GPSDataStorage::WGSContainer& GPSDataStorage::getWGSContainer()
    const {
  return wgs_data_;
}

inline const GPSDataStorage::UTMContainer& GPSDataStorage::getUTMContainer()
    const {
  return utm_data_;
}

inline bool GPSDataStorage::hasWGSBufferForMission(
    const MissionId& mission_id) const {
  CHECK(mission_id.isValid());
  return wgs_data_.count(mission_id) > 0u;
}

inline bool GPSDataStorage::hasUTMBufferForMission(
    const MissionId& mission_id) const {
  CHECK(mission_id.isValid());
  return utm_data_.count(mission_id) > 0u;
}

inline const GPSDataStorage::WGSBuffer& GPSDataStorage::getWGSBufferForMission(
    const MissionId& mission_id) const {
  CHECK(mission_id.isValid());
  CHECK(hasWGSBufferForMission(mission_id));
  return wgs_data_.at(mission_id);
}

inline const GPSDataStorage::UTMBuffer& GPSDataStorage::getUTMBufferForMission(
    const MissionId& mission_id) const {
  CHECK(mission_id.isValid());
  CHECK(hasUTMBufferForMission(mission_id));
  return utm_data_.at(mission_id);
}

}  // namespace vi_map

#endif  // VI_MAP_DEPRECATED_GPS_DATA_STORAGE_INL_H_
