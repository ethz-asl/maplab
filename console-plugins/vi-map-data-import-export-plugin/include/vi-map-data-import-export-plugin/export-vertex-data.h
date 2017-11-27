#ifndef VI_MAP_DATA_IMPORT_EXPORT_PLUGIN_EXPORT_VERTEX_DATA_H_
#define VI_MAP_DATA_IMPORT_EXPORT_PLUGIN_EXPORT_VERTEX_DATA_H_

#include <string>

#include <vi-map/vi-map.h>

namespace data_import_export {

int exportPosesVelocitiesAndBiasesToCsv(
    const vi_map::VIMap& map, const vi_map::MissionIdList& mission_ids,
    const std::string& pose_export_file);

}  // namespace data_import_export

#endif  // VI_MAP_DATA_IMPORT_EXPORT_PLUGIN_EXPORT_VERTEX_DATA_H_
