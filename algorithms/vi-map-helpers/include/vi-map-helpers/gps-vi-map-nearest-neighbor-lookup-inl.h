#ifndef VI_MAP_HELPERS_GPS_VI_MAP_NEAREST_NEIGHBOR_LOOKUP_INL_H_
#define VI_MAP_HELPERS_GPS_VI_MAP_NEAREST_NEIGHBOR_LOOKUP_INL_H_

#include <glog/logging.h>

namespace vi_map_helpers {

template <>
inline size_t getQueryTypeDimension<vi_map::GpsUtmMeasurement>() {
  return 3u;
}

template <>
inline size_t getQueryTypeDimension<vi_map::GpsWgsMeasurement>() {
  return 2u;
}

template <>
inline Eigen::VectorXd queryTypeToVector(
    const vi_map::GpsWgsMeasurement& wgs_measurement) {
  return Eigen::Vector2d(
      wgs_measurement.getLatitudeDeg(), wgs_measurement.getLongitudeDeg());
}

template <>
inline Eigen::VectorXd queryTypeToVector(
    const vi_map::GpsUtmMeasurement& utm_measurement) {
  return utm_measurement.get_T_UTM_S().getPosition();
}

template <>
inline vi_map::GpsWgsMeasurement
createDataItem<vi_map::GpsWgsMeasurement, vi_map::GpsWgsMeasurement>(
    const vi_map::GpsWgsMeasurement& gps_measuremnt,
    const vi_map::MissionId& /*mission_id*/) {
  return gps_measuremnt;
}

template <>
inline vi_map::GpsWgsMeasurementMissionPair
createDataItem<vi_map::GpsWgsMeasurement, vi_map::GpsWgsMeasurementMissionPair>(
    const vi_map::GpsWgsMeasurement& gps_measuremnt,
    const vi_map::MissionId& mission_id) {
  CHECK(mission_id.isValid());
  return vi_map::GpsWgsMeasurementMissionPair(gps_measuremnt, mission_id);
}

template <>
inline vi_map::GpsUtmMeasurement
createDataItem<vi_map::GpsUtmMeasurement, vi_map::GpsUtmMeasurement>(
    const vi_map::GpsUtmMeasurement& gps_measuremnt,
    const vi_map::MissionId& /*mission_id*/) {
  return gps_measuremnt;
}

template <>
inline vi_map::GpsUtmMeasurementMissionPair
createDataItem<vi_map::GpsUtmMeasurement, vi_map::GpsUtmMeasurementMissionPair>(
    const vi_map::GpsUtmMeasurement& gps_measuremnt,
    const vi_map::MissionId& mission_id) {
  CHECK(mission_id.isValid());
  return vi_map::GpsUtmMeasurementMissionPair(gps_measuremnt, mission_id);
}

}  // namespace vi_map_helpers

#endif  // VI_MAP_HELPERS_GPS_VI_MAP_NEAREST_NEIGHBOR_LOOKUP_INL_H_
