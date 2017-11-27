#ifndef VI_MAP_DATA_IMPORT_EXPORT_PLUGIN_IMPORT_EXPORT_GPS_DATA_H_
#define VI_MAP_DATA_IMPORT_EXPORT_PLUGIN_IMPORT_EXPORT_GPS_DATA_H_

#include <string>

#include <vi-map/vi-map.h>

namespace data_import_export {

void importGpsDataFromRosbag(
    const std::string& bag_filename, const std::string& gps_topic,
    const std::string& gps_yaml, const vi_map::MissionId& mission_id,
    vi_map::VIMap* map);

template <typename GpsMeasurement>
void exportGpsDataMatchedToVerticesToCsv(
    const vi_map::VIMap& map, const std::string& csv_filename);

template <typename GpsMeasurement>
void convertGpsMeasurementToCsvLine(const GpsMeasurement& utm_measurement,
                                    std::string* csv_line);

template <typename GpsMeasurement>
void getCsvHeaderLine(std::string* csv_header_line);

template <typename GpsMeasurement>
inline bool hasData();

const char kDelimiter = ',';

namespace internal {
template <class GpsMeasurement>
vi_map::SensorType getSensorTypeForMeasurement();
}  // namespace internal

}  // namespace data_import_export

#include "./import-export-gps-data-inl.h"

#endif  // VI_MAP_DATA_IMPORT_EXPORT_PLUGIN_IMPORT_EXPORT_GPS_DATA_H_
