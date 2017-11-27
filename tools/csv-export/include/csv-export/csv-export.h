#ifndef CSV_EXPORT_CSV_EXPORT_H_
#define CSV_EXPORT_CSV_EXPORT_H_
#include <string>
#include <vector>

#include <Eigen/Core>
#include <vi-map/vi-map.h>

namespace csv_export {

void exportMapToCsv(
    const vi_map::VIMap& map, const std::string& export_base_filename);

}  // namespace csv_export

#endif  // CSV_EXPORT_CSV_EXPORT_H_
