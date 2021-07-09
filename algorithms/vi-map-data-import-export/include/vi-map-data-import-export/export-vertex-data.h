#ifndef VI_MAP_DATA_IMPORT_EXPORT_EXPORT_VERTEX_DATA_H_
#define VI_MAP_DATA_IMPORT_EXPORT_EXPORT_VERTEX_DATA_H_

#include <string>

#include <aslam/common/pose-types.h>
#include <aslam/common/unique-id.h>
#include <vi-map/vi-map.h>

namespace data_import_export {

int exportPosesVelocitiesAndBiasesToCsv(
    const vi_map::VIMap& map, const vi_map::MissionIdList& mission_ids,
    const aslam::SensorId& reference_sensor_id,
    const std::string& pose_export_file,
    const std::string& format = std::string("asl"));
static const double kNanoSecondsToSeconds = 1e-9;
}  // namespace data_import_export

#endif  // VI_MAP_DATA_IMPORT_EXPORT_EXPORT_VERTEX_DATA_H_
