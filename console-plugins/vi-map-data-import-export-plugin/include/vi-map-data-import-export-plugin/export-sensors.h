#ifndef VI_MAP_DATA_IMPORT_EXPORT_PLUGIN_EXPORT_SENSORS_H_
#define VI_MAP_DATA_IMPORT_EXPORT_PLUGIN_EXPORT_SENSORS_H_

#include <string>

#include <vi-map/vi-map.h>

namespace data_import_export {

int exportSensors(const vi_map::VIMap& map, const std::string& output_folder);

}  // namespace data_import_export

#endif  // VI_MAP_DATA_IMPORT_EXPORT_PLUGIN_EXPORT_SENSORS_H_
