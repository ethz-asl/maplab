#include "vi-map-data-import-export-plugin/export-sensors.h"

#include <console-common/command-registerer.h>
#include <maplab-common/file-system-tools.h>
#include <vi-map/vi-map.h>

namespace data_import_export {

int exportSensors(const vi_map::VIMap& map, const std::string& output_folder) {
  CHECK(!output_folder.empty());
  if (!common::createPath(output_folder)) {
    LOG(ERROR) << "Failed to create folder: " << output_folder << ".";
    return common::kStupidUserError;
  }

  const vi_map::SensorManager& sensor_manager = map.getSensorManager();
  vi_map::SensorIdSet sensor_ids;
  sensor_manager.getAllSensorIds(&sensor_ids);

  for (const vi_map::SensorId& sensor_id : sensor_ids) {
    CHECK(sensor_id.isValid());

    std::string filename;
    common::concatenateFolderAndFileName(
        output_folder, sensor_id.hexString() + ".yaml", &filename);
    LOG(INFO) << "Exporting sensor to " << filename << ".";

    const vi_map::Sensor& sensor = sensor_manager.getSensor(sensor_id);
    sensor.serializeToFile(filename);
  }
  return common::kSuccess;
}

}  // namespace data_import_export
