#ifndef MAPLAB_NODE_DATASOURCE_FACTORY_H_
#define MAPLAB_NODE_DATASOURCE_FACTORY_H_

#include <string>

#include <glog/logging.h>
#include <sensors/imu.h>
#include <vio-common/rostopic-settings.h>

#include "maplab-node/datasource.h"

namespace maplab {
enum class DataSourceType { kRosTopic, kRosBag };

DataSourceType stringToDataSource(const std::string& str);

DataSource* createAndConfigureDataSourcefromGflagsAndTopics(
    const vio_common::RosTopicSettings& ros_topic_settings,
    const vi_map::SensorManager& sensor_manager);

}  // namespace maplab
#endif  // MAPLAB_NODE_DATASOURCE_FACTORY_H_
