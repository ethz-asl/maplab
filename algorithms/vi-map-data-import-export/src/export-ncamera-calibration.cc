#include "vi-map-data-import-export/export-ncamera-calibration.h"

#include <unordered_set>

#include <aslam/cameras/ncamera.h>
#include <aslam/common/unique-id.h>
#include <console-common/command-registerer.h>
#include <maplab-common/file-system-tools.h>
#include <sensors/sensor-types.h>
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

  aslam::NCameraIdSet ncamera_ids;
  map.getSensorManager().getAllSensorIdsOfType(
      vi_map::SensorType::kNCamera, &ncamera_ids);

  for (const aslam::NCameraId& ncamera_id : ncamera_ids) {
    CHECK(ncamera_id.isValid());
    std::string filename;
    common::concatenateFolderAndFileName(
        output_folder, ncamera_id.hexString() + ".yaml", &filename);
    LOG(INFO) << "Exporting ncamera calibration to " << filename << ".";
    const aslam::NCamera& ncamera =
        map.getSensorManager().getSensor<aslam::NCamera>(ncamera_id);
    ncamera.serializeToFile(filename);
  }

  LOG(INFO) << "Saved " << ncamera_ids.size() << " NCamera calibration files.";
  return common::kSuccess;
}

}  // namespace data_import_export
