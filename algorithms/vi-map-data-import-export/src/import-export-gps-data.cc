#include "vi-map-data-import-export/import-export-gps-data.h"

#include <aslam/common/time.h>
#include <glog/logging.h>
#include <maplab-common/progress-bar.h>
#include <nav_msgs/Odometry.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensors/sensor-factory.h>
#include <vi-map/vi-map.h>

namespace data_import_export {

inline int64_t rosTimeToNanoseconds(const ros::Time& rostime) {
  return aslam::time::from_seconds(static_cast<int64_t>(rostime.sec)) +
         static_cast<int64_t>(rostime.nsec);
}

bool processGpsWgsMeasurement(
    const rosbag::MessageInstance& message, const vi_map::SensorId& sensor_id,
    vi_map::OptionalSensorData* const optional_sensor_data) {
  CHECK_NOTNULL(optional_sensor_data);
  sensor_msgs::NavSatFixConstPtr gps_wgs_message =
      message.instantiate<sensor_msgs::NavSatFix>();
  if (!gps_wgs_message) {
    LOG(ERROR) << "The GPS message is not of type NavSatFix as suggested "
               << "by the sensor type found in the specified YAML file. "
               << "Please make sure the messages of the specified rostopic "
               << "are of type NavSatFix.";
    return false;
  }

  const vi_map::GpsWgsMeasurement gps_wgs_measurement(
      sensor_id, rosTimeToNanoseconds(gps_wgs_message->header.stamp),
      gps_wgs_message->latitude, gps_wgs_message->longitude,
      gps_wgs_message->altitude);

  optional_sensor_data->addMeasurement(gps_wgs_measurement);

  return true;
}

bool processGpsUtmMeasurement(
    const rosbag::MessageInstance& message, const vi_map::SensorId& sensor_id,
    vi_map::OptionalSensorData* const optional_sensor_data) {
  CHECK_NOTNULL(optional_sensor_data);

  nav_msgs::OdometryConstPtr gps_utm_message =
      message.instantiate<nav_msgs::Odometry>();
  if (!gps_utm_message) {
    LOG(ERROR) << "The GPS message is not of type Odometry as suggested "
               << "by the sensor type found in the specified YAML file. "
               << "Please make sure the messages of the specified rostopic "
               << "are of type Odometry.";
    return false;
  }

  const aslam::Quaternion q_R_S(
      gps_utm_message->pose.pose.orientation.w,
      gps_utm_message->pose.pose.orientation.x,
      gps_utm_message->pose.pose.orientation.y,
      gps_utm_message->pose.pose.orientation.z);
  const aslam::Position3D p_R_S(
      gps_utm_message->pose.pose.position.x,
      gps_utm_message->pose.pose.position.y,
      gps_utm_message->pose.pose.position.z);

  const vi_map::GpsUtmMeasurement gps_utm_measurement(
      sensor_id, rosTimeToNanoseconds(gps_utm_message->header.stamp),
      aslam::Transformation(q_R_S, p_R_S), vi_map::UtmZone::createInvalid());

  optional_sensor_data->addMeasurement(gps_utm_measurement);

  return true;
}

template <>
void addCsvHeaderFields<vi_map::GpsUtmMeasurement>(
    std::vector<std::string>* csv_header_fields) {
  CHECK_NOTNULL(csv_header_fields);
  csv_header_fields->emplace_back("GPS UTM timestamp [ns]");
  csv_header_fields->emplace_back("p_UTM_B_x [m]");
  csv_header_fields->emplace_back("p_UTM_B_y [m]");
  csv_header_fields->emplace_back("p_UTM_B_z [m]");
  csv_header_fields->emplace_back("q_UTM_B_w");
  csv_header_fields->emplace_back("q_UTM_B_z");
  csv_header_fields->emplace_back("q_UTM_B_y");
  csv_header_fields->emplace_back("q_UTM_B_z");
}

template <>
void addCsvHeaderFields<vi_map::GpsWgsMeasurement>(
    std::vector<std::string>* csv_header_fields) {
  CHECK_NOTNULL(csv_header_fields);
  csv_header_fields->emplace_back("GPS WGS timestamp [ns]");
  csv_header_fields->emplace_back("latidude [deg]");
  csv_header_fields->emplace_back("longitude [deg]");
  csv_header_fields->emplace_back("altitude [m]");
}

template <>
void convertGpsMeasurementToCsvFields<vi_map::GpsUtmMeasurement>(
    const vi_map::GpsUtmMeasurement& utm_measurement,
    std::vector<std::string>* csv_fields) {
  CHECK_NOTNULL(csv_fields)->clear();
  *csv_fields = {
      std::to_string(utm_measurement.getTimestampNanoseconds()),
      std::to_string(utm_measurement.get_T_UTM_S().getPosition()[0]),
      std::to_string(utm_measurement.get_T_UTM_S().getPosition()[1]),
      std::to_string(utm_measurement.get_T_UTM_S().getPosition()[2]),
      std::to_string(utm_measurement.get_T_UTM_S().getRotation().w()),
      std::to_string(utm_measurement.get_T_UTM_S().getRotation().x()),
      std::to_string(utm_measurement.get_T_UTM_S().getRotation().y()),
      std::to_string(utm_measurement.get_T_UTM_S().getRotation().z())};
}

template <>
void convertGpsMeasurementToCsvFields<vi_map::GpsWgsMeasurement>(
    const vi_map::GpsWgsMeasurement& wgs_measurement,
    std::vector<std::string>* csv_fields) {
  CHECK_NOTNULL(csv_fields)->clear();
  *csv_fields = {std::to_string(wgs_measurement.getTimestampNanoseconds()),
                 std::to_string(wgs_measurement.getLatitudeDeg()),
                 std::to_string(wgs_measurement.getLongitudeDeg()),
                 std::to_string(wgs_measurement.getAltitudeMeters())};
}

void importGpsDataFromRosbag(
    const std::string& bag_filename, const std::string& gps_topic,
    const std::string& gps_yaml, const vi_map::MissionId& mission_id,
    vi_map::VIMap* map) {
  CHECK_NOTNULL(map);
  CHECK(map->hasMission(mission_id));
  CHECK(common::fileExists(gps_yaml));
  CHECK(!gps_topic.empty());

  rosbag::Bag bag;

  try {
    bag.open(bag_filename, rosbag::bagmode::Read);
  } catch (const rosbag::BagException& bag_exception) {  // NOLINT
    LOG(FATAL) << "Could not open the rosbag " << bag_filename << ": "
               << bag_exception.what();
  }

  std::vector<std::string> topics;
  topics.emplace_back(gps_topic);

  vi_map::Sensor::UniquePtr sensor = vi_map::createSensorFromYaml(gps_yaml);
  CHECK(sensor);
  const vi_map::SensorType sensor_type = sensor->getSensorType();
  CHECK(
      sensor_type == vi_map::SensorType::kGpsUtm ||
      sensor_type == vi_map::SensorType::kGpsWgs);
  const vi_map::SensorId gps_sensor_id = sensor->getId();
  CHECK(gps_sensor_id.isValid());
  map->getSensorManager().addSensor(std::move(sensor), mission_id);

  rosbag::View view(bag, rosbag::TopicQuery(topics));

  const size_t num_messages = view.size();
  if (num_messages == 0u) {
    LOG(ERROR) << "The bag view contains zero messages. "
               << "Selected GPS topic: " << gps_topic;
    return;
  }

  vi_map::OptionalSensorData& optional_sensor_data =
      map->getOptionalSensorData(mission_id);

  std::function<bool(
      const rosbag::MessageInstance& message, const vi_map::SensorId& sensor_id,
      vi_map::OptionalSensorData* optional_sensor_data)>
      process_gps_measurement;

  switch (sensor_type) {
    case vi_map::SensorType::kGpsWgs:
      process_gps_measurement = &processGpsWgsMeasurement;
      optional_sensor_data.clear<vi_map::GpsWgsMeasurement>(gps_sensor_id);
      break;
    case vi_map::SensorType::kGpsUtm:
      process_gps_measurement = &processGpsUtmMeasurement;
      optional_sensor_data.clear<vi_map::GpsUtmMeasurement>(gps_sensor_id);
      break;
    default:
      LOG(ERROR) << "Unknown sensor type in GPS YAML file.";
      return;
  }

  common::ProgressBar progress_bar(num_messages);
  size_t num_added = 0u;
  for (rosbag::MessageInstance& message : view) {
    CHECK_EQ(message.getTopic(), gps_topic);
    if (!process_gps_measurement(
            message, gps_sensor_id, &optional_sensor_data)) {
      LOG(ERROR) << "Unable to process message. Aborting. Added " << num_added
                 << " GPS measurements.";
      break;
    }
    ++num_added;
    progress_bar.increment();
  }
}

namespace internal {

template <>
vi_map::SensorType getSensorTypeForMeasurement<vi_map::GpsUtmMeasurement>() {
  return vi_map::SensorType::kGpsUtm;
}

template <>
vi_map::SensorType getSensorTypeForMeasurement<vi_map::GpsWgsMeasurement>() {
  return vi_map::SensorType::kGpsWgs;
}

}  // namespace internal

}  // namespace data_import_export
