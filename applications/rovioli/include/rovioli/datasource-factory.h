#ifndef ROVIOLI_DATASOURCE_FACTORY_H_
#define ROVIOLI_DATASOURCE_FACTORY_H_

#include <string>

#include <glog/logging.h>
#include <sensors/imu.h>

#include "rovioli/datasource.h"

namespace rovioli {
enum class DataSourceType { kRosTopic, kRosBag };

DataSourceType stringToDataSource(const std::string& str);

// Camera topics are read from the camera calibration file. Caller takes
// ownership!
DataSource* createAndConfigureDataSourcefromGFlags(
    const aslam::NCamera& camera_system, const vi_map::Imu& imu_sensor);
}  // namespace rovioli
#endif  // ROVIOLI_DATASOURCE_FACTORY_H_
