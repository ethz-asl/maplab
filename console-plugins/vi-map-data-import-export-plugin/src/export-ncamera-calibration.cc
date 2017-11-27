#include "vi-map-data-import-export-plugin/export-ncamera-calibration.h"

#include <unordered_set>

#include <aslam/cameras/ncamera.h>
#include <aslam/common/unique-id.h>
#include <console-common/command-registerer.h>
#include <maplab-common/file-system-tools.h>
#include <vi-map/unique-id.h>
#include <vi-map/vi-map.h>
#include <vi-map/vi-mission.h>

namespace data_import_export {

int exportNCameraCalibration(
    const vi_map::VIMap& map, const std::string& output_folder) {
  CHECK(!output_folder.empty());
  if (!common::createPath(output_folder)) {
    LOG(ERROR) << "Failed to create folder: " << output_folder << ".";
    return common::kStupidUserError;
  }

  const vi_map::SensorManager& sensor_manager = map.getSensorManager();

  aslam::NCameraIdSet ncamera_ids;
  sensor_manager.getAllNCameraIds(&ncamera_ids);

  for (const aslam::NCameraId& ncamera_id : ncamera_ids) {
    CHECK(ncamera_id.isValid());
    std::string filename;
    common::concatenateFolderAndFileName(
        output_folder, ncamera_id.hexString() + ".yaml", &filename);
    LOG(INFO) << "Exporting ncamera calibration to " << filename << ".";
    const aslam::NCamera& ncamera = sensor_manager.getNCamera(ncamera_id);
    CHECK(ncamera.saveToYaml(filename));
  }

  LOG(INFO) << "Saved " << ncamera_ids.size() << " NCamera calibration files.";
  return common::kSuccess;
}

}  // namespace data_import_export
