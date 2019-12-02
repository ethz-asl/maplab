#ifndef ROVIOLI_DATASOURCE_FACTORY_H_
#define ROVIOLI_DATASOURCE_FACTORY_H_

#include <string>

#include <glog/logging.h>
#include <sensors/imu.h>
#include <vio-common/rostopic-settings.h>

#include "rovioli/datasource.h"

namespace rovioli {
enum class DataSourceType { kRosTopic, kRosBag };

DataSourceType stringToDataSource(const std::string& str);

// Camera topics are read from the camera calibration file. Caller takes
// ownership!
DataSource* createAndConfigureDataSourcefromGFlags(
    const vio_common::RosTopicSettings& topic_settings);
}  // namespace rovioli
#endif  // ROVIOLI_DATASOURCE_FACTORY_H_
