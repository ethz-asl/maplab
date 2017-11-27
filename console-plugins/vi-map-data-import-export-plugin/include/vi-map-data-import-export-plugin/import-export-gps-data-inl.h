#ifndef VI_MAP_DATA_IMPORT_EXPORT_PLUGIN_IMPORT_EXPORT_GPS_DATA_INL_H_
#define VI_MAP_DATA_IMPORT_EXPORT_PLUGIN_IMPORT_EXPORT_GPS_DATA_INL_H_

#include <string>

#include <aslam/common/time.h>
#include <glog/logging.h>
#include <maplab-common/file-logger.h>
#include <vi-map/vi-map.h>

namespace data_import_export {

template <typename GpsMeasurement>
void exportGpsDataMatchedToVerticesToCsv(
    const vi_map::VIMap& map, const std::string& csv_filename) {
  CHECK(!csv_filename.empty());

  vi_map::MissionIdList mission_ids;
  map.getAllMissionIds(&mission_ids);

  if (mission_ids.empty()) {
    LOG(ERROR) << "Zero missions available. Aborting.";
    return;
  }

  const std::string csv_filepath =
      common::concatenateFolderAndFileName(map.getMapFolder(), csv_filename);

  common::FileLogger file(csv_filepath);
  if (!file.isOpen()) {
    LOG(ERROR) << "Unable to open file " << csv_filepath << " for writing.";
    return;
  }

  std::string header_line;
  getCsvHeaderLine<GpsMeasurement>(&header_line);
  file << "# vertex timestamp [ns]" << kDelimiter << " vertex id" << kDelimiter
       << " UTM timestamp [ns], " << header_line;

  const int64_t kTimestampToleranceNanoseconds = aslam::time::milliseconds(10);

  const vi_map::SensorManager& sensor_manager = map.getSensorManager();
  for (const vi_map::MissionId& mission_id : mission_ids) {
    CHECK(mission_id.isValid());

    vi_map::SensorIdSet gps_sensor_ids;
    sensor_manager.getAllSensorIdsOfTypeAssociatedWithMission(
        internal::getSensorTypeForMeasurement<GpsMeasurement>(), mission_id,
        &gps_sensor_ids);
    if (gps_sensor_ids.empty()) {
      LOG(INFO) << "There is no GPS sensor registered for mission "
                << mission_id.hexString()
                << "; will skip exporting GPS data of "
                << "this mission.";
      continue;
    }

    CHECK(map.hasOptionalSensorData(mission_id))
        << "The given map is inconsistent. At least one GPS sensor is "
        << "registered for mission " << mission_id.hexString()
        << ", yet, there is no sensor data available.";

    if (gps_sensor_ids.size() > 1u) {
      LOG(ERROR) << "More than one GPS sensor registered for mission "
                 << mission_id.hexString()
                 << ". This case is not supported yet. Skipping this mission";
      continue;
    }

    const vi_map::SensorId& gps_sensor_id = *gps_sensor_ids.begin();
    CHECK(gps_sensor_id.isValid());
    const vi_map::MeasurementBuffer<GpsMeasurement>& gps_utm_measurements =
        map.getOptionalSensorMeasurements<GpsMeasurement>(
            gps_sensor_id, mission_id);

    VLOG(1) << "Exporting GPS UTM measurements of mission "
            << mission_id.hexString() << '.';

    pose_graph::VertexIdList vertex_ids;
    map.getAllVertexIdsInMissionAlongGraph(mission_id, &vertex_ids);

    for (const pose_graph::VertexId& vertex_id : vertex_ids) {
      CHECK(vertex_id.isValid());
      const int64_t timestamp_nanosecond =
          map.getVertex(vertex_id).getMinTimestampNanoseconds();

      GpsMeasurement gps_measurement;
      if (gps_utm_measurements.getNearestValueToTime(
              timestamp_nanosecond, &gps_measurement)) {
        if (std::abs(
                gps_measurement.getTimestampNanoseconds() -
                timestamp_nanosecond) > kTimestampToleranceNanoseconds) {
          LOG(WARNING)
              << "Unable to find a matching UTM measurement for vertex "
              << vertex_id.hexString() << " of mission "
              << mission_id.hexString() << " within "
              << kTimestampToleranceNanoseconds << "ns timestamp tolerance.";
          continue;
        }
        std::string csv_line;
        convertGpsMeasurementToCsvLine(gps_measurement, &csv_line);
        file << std::to_string(timestamp_nanosecond) << kDelimiter
             << vertex_id.hexString() << kDelimiter << csv_line << std::endl;
      } else {
        LOG(WARNING)
            << "Unable to find matching GPS UTM measurement for vertex "
            << vertex_id.hexString();
      }
    }
  }
}

}  // namespace data_import_export

#endif  // VI_MAP_DATA_IMPORT_EXPORT_PLUGIN_IMPORT_EXPORT_GPS_DATA_INL_H_
