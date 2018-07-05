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

template <typename GpsQueryType, typename GpsDataType>
void GpsVIMapNearestNeighborLookup<GpsQueryType, GpsDataType>::buildIndexImpl(
    const vi_map::VIMap& map) {
  using GpsMeasurementBuffer = vi_map::MeasurementBuffer<GpsQueryType>;

  const vi_map::SensorType kSensorType =
      vi_map::measurementToSensorType<GpsQueryType>();

  const vi_map::SensorManager& sensor_manager = map.getSensorManager();
  vi_map::SensorIdSet gps_sensor_ids;
  sensor_manager.getAllSensorIdsOfType(kSensorType, &gps_sensor_ids);
  VLOG(2) << "The map has " << gps_sensor_ids.size() << " sensors of type "
          << vi_map::sensorTypeToString(kSensorType);

  vi_map::MissionIdList all_mission_ids;
  map.getAllMissionIds(&all_mission_ids);

  size_t total_num_gps_measurements = 0u;
  for (const vi_map::MissionId& mission_id : all_mission_ids) {
    CHECK(mission_id.isValid());
    if (!map.hasOptionalSensorData(mission_id)) {
      continue;
    }

    const vi_map::OptionalSensorData& optional_sensor_data =
        map.getOptionalSensorData(mission_id);

    optional_sensor_data.lock();

    for (const vi_map::SensorId& gps_sensor_id : gps_sensor_ids) {
      CHECK(gps_sensor_id.isValid());
      const GpsMeasurementBuffer& gps_measurements =
          optional_sensor_data.getMeasurements<GpsQueryType>(gps_sensor_id);
      total_num_gps_measurements += gps_measurements.size();
    }
  }

  if (total_num_gps_measurements > 0u) {
    VLOG(2) << "Building GPS index with " << total_num_gps_measurements
            << " measurements.";

    const size_t kQueryDimVector = getQueryTypeDimension<GpsQueryType>();
    BaseType::resizeIndexData(kQueryDimVector, total_num_gps_measurements);

    size_t col_idx = 0u;
    for (const vi_map::MissionId& mission_id : all_mission_ids) {
      CHECK(mission_id.isValid());
      const vi_map::OptionalSensorData& optional_sensor_data =
          map.getOptionalSensorData(mission_id);
      for (const vi_map::SensorId& gps_sensor_id : gps_sensor_ids) {
        CHECK(gps_sensor_id.isValid());
        const GpsMeasurementBuffer& gps_measurements =
            optional_sensor_data.getMeasurements<GpsQueryType>(gps_sensor_id);

        for (const typename GpsMeasurementBuffer::BufferType::value_type&
                 time_gps_pair : gps_measurements) {
          const GpsDataType& data_item =
              createDataItem<GpsQueryType, GpsDataType>(
                  time_gps_pair.second, mission_id);
          BaseType::emplaceBackDataItem(data_item);
          const Eigen::VectorXd data_item_as_vector =
              queryTypeToVector(time_gps_pair.second);
          BaseType::setNearestNeighborIndexDataColumn(
              queryTypeToVector(time_gps_pair.second), col_idx);
          ++col_idx;
        }
      }

      optional_sensor_data.unlock();
    }
    BaseType::initializeIndex();
  }

  for (const vi_map::MissionId& mission_id : all_mission_ids) {
    CHECK(mission_id.isValid());
    if (!map.hasOptionalSensorData(mission_id)) {
      continue;
    }

    const vi_map::OptionalSensorData& optional_sensor_data =
        map.getOptionalSensorData(mission_id);

    optional_sensor_data.unlock();
  }
}

}  // namespace vi_map_helpers

#endif  // VI_MAP_HELPERS_GPS_VI_MAP_NEAREST_NEIGHBOR_LOOKUP_INL_H_
