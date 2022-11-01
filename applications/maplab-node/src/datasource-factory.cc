#include "maplab-node/datasource-factory.h"

#include <string>

#include <aslam/cameras/ncamera.h>
#include <gflags/gflags.h>
#include <glog/logging.h>
#include <maplab-common/file-system-tools.h>
#include <vio-common/rostopic-settings.h>

#include "maplab-node/datasource-rosbag.h"
#include "maplab-node/datasource-rostopic.h"

DEFINE_string(datasource_type, "rosbag", "Data source type: rosbag / rostopic");
DEFINE_string(datasource_rosbag, "", "Path to rosbag for bag sources.");

namespace maplab {

DataSourceType stringToDataSource(const std::string& str) {
  if (str == "rostopic") {
    return DataSourceType::kRosTopic;
  } else if (str == "rosbag") {
    return DataSourceType::kRosBag;
  }
  LOG(FATAL) << "Unknown datasource: " << str;
  return DataSourceType::kRosBag;  // Silence warning.
}

DataSource* createAndConfigureDataSourcefromGflagsAndTopics(
    const vio_common::RosTopicSettings& topic_settings,
    const vi_map::SensorManager& sensor_manager) {
  const DataSourceType source_type = stringToDataSource(FLAGS_datasource_type);
  switch (source_type) {
    case DataSourceType::kRosBag:
      CHECK(!FLAGS_datasource_rosbag.empty());
      CHECK(common::fileExists(FLAGS_datasource_rosbag));
      return new DataSourceRosbag(
          FLAGS_datasource_rosbag, topic_settings, sensor_manager);
      break;
    case DataSourceType::kRosTopic:
      return new DataSourceRostopic(topic_settings, sensor_manager);
      break;
    default:
      LOG(FATAL);
      break;
  }
  return nullptr;
}

}  // namespace maplab
