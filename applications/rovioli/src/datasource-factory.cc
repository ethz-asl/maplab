#include "rovioli/datasource-factory.h"

#include <string>

#include <aslam/cameras/ncamera.h>
#include <gflags/gflags.h>
#include <glog/logging.h>
#include <maplab-common/file-system-tools.h>
#include <vio-common/rostopic-settings.h>

#include "rovioli/datasource-rosbag.h"
#include "rovioli/datasource-rostopic.h"

DEFINE_string(datasource_type, "rosbag", "Data source type: rosbag / rostopic");
DEFINE_string(datasource_rosbag, "", "Path to rosbag for bag sources.");

namespace rovioli {

DataSourceType stringToDataSource(const std::string& str) {
  if (str == "rostopic") {
    return DataSourceType::kRosTopic;
  } else if (str == "rosbag") {
    return DataSourceType::kRosBag;
  }
  LOG(FATAL) << "Unknown datasource: " << str;
  return DataSourceType::kRosBag;  // Silence warning.
}

DataSource* createAndConfigureDataSourcefromGFlags(
    const aslam::NCamera& camera_system, const vi_map::Imu& imu_sensor) {
  const vio_common::RosTopicSettings topic_settings(camera_system, imu_sensor);
  const DataSourceType source_type = stringToDataSource(FLAGS_datasource_type);
  switch (source_type) {
    case DataSourceType::kRosBag:
      CHECK(!FLAGS_datasource_rosbag.empty());
      CHECK(common::fileExists(FLAGS_datasource_rosbag));
      return new DataSourceRosbag(FLAGS_datasource_rosbag, topic_settings);
      break;
    case DataSourceType::kRosTopic:
      return new DataSourceRostopic(topic_settings);
      break;
    default:
      LOG(FATAL);
      break;
  }
  return nullptr;
}
}  // namespace rovioli
