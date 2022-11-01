#include "vi-map/sensor-utils.h"

#include <memory>

#include <aslam/common/sensor.h>
#include <sensors/imu.h>
#include <sensors/lidar.h>
#include <sensors/sensor-types.h>
#include <sensors/wheel-odometry-sensor.h>

#include "vi-map/sensor-manager.h"

DEFINE_string(
    selected_ncamera_sensor_id, "",
    "If there is more than one NCamera in the sensor manager, use this flag "
    "to determine which one is used.");

DEFINE_string(
    selected_imu_sensor_id, "",
    "If there is more than one IMU in the sensor manager, use this flag "
    "to determine which one is used.");

DEFINE_string(
    selected_lidar_sensor_id, "",
    "If there is more than one Lidar in the sensor manager, use this flag "
    "to determine which one is used.");

DEFINE_string(
    selected_odometry_6dof_sensor_id, "",
    "If there is more than one 6DOF Odometry sensor in the sensor manager, use "
    "this flag to determine which one is used.");

DEFINE_string(
    selected_loop_closure_sensor_id, "",
    "If there is more than one Relative6DOF sensor in the sensor manager, use "
    "this flag to determine which one is used.");

DEFINE_string(
    selected_absolute_6dof_sensor_id, "",
    "If there is more than one Absolute6DoF sensor in the sensor manager, use "
    "this flag to determine which one is used.");

DEFINE_string(
    selected_wheel_odometry_sensor_id, "",
    "If there is more than one WheelOdometry sensor in the sensor manager, use "
    "this flag to determine which one is used.");

DEFINE_string(
    selected_gps_utm_sensor_id, "",
    "If there is more than one GPS UTM sensor in the sensor manager, use "
    "this flag to determine which one is used.");

DEFINE_string(
    selected_gps_wgs_sensor_id, "",
    "If there is more than one GPS WGS sensor in the sensor manager, use "
    "this flag to determine which one is used.");

DEFINE_string(
    selected_point_cloud_map_sensor_id, "",
    "If there is more than one point cloud map sensor in the sensor manager, "
    "use this flag to determine which one is used.");

namespace vi_map {

aslam::NCamera::Ptr getSelectedNCamera(const SensorManager& sensor_manager) {
  aslam::SensorIdSet all_ncamera_ids;
  sensor_manager.getAllSensorIdsOfType(SensorType::kNCamera, &all_ncamera_ids);

  if (all_ncamera_ids.empty()) {
    VLOG(3) << "No NCameras were found in sensor manager!";
    return aslam::NCamera::Ptr();
  }

  aslam::SensorId mapping_ncamera_id;
  if (all_ncamera_ids.size() > 1u) {
    CHECK(
        !FLAGS_selected_ncamera_sensor_id.empty() &&
        mapping_ncamera_id.fromHexString(FLAGS_selected_ncamera_sensor_id))
        << "If more than one NCamera is provided in the sensor manager, use "
        << "--selected_ncamera_sensor_id to select which one to use.";

    CHECK(
        sensor_manager.hasSensor(mapping_ncamera_id) &&
        (sensor_manager.getSensorType(mapping_ncamera_id) ==
         SensorType::kNCamera))
        << "The sensor id provided by --selected_ncamera_sensor_id is not "
        << "in the sensor manager or is not an NCamera!";
  } else {
    mapping_ncamera_id = *all_ncamera_ids.begin();
  }

  aslam::NCamera::Ptr mapping_ncamera_ptr =
      sensor_manager.getSensorPtr<aslam::NCamera>(mapping_ncamera_id);
  CHECK(mapping_ncamera_ptr);
  return mapping_ncamera_ptr;
}

Imu::Ptr getSelectedImu(const SensorManager& sensor_manager) {
  aslam::SensorIdSet all_imu_ids;
  sensor_manager.getAllSensorIdsOfType(SensorType::kImu, &all_imu_ids);

  if (all_imu_ids.empty()) {
    VLOG(3) << "No Imu was found in the sensor manager!";
    return Imu::Ptr();
  }

  aslam::SensorId mapping_imu_id;
  if (all_imu_ids.size() > 1u) {
    CHECK(
        !FLAGS_selected_imu_sensor_id.empty() &&
        mapping_imu_id.fromHexString(FLAGS_selected_imu_sensor_id))
        << "If more than one Imu is provided in the sensor manager, use "
        << "--selected_imu_sensor_id to select which one to use.";

    CHECK(
        sensor_manager.hasSensor(mapping_imu_id) &&
        (sensor_manager.getSensorType(mapping_imu_id) == SensorType::kImu))
        << "The sensor id provided by --selected_imu_sensor_id is not "
        << "in the sensor manager or is not an Imu!";
  } else {
    mapping_imu_id = *all_imu_ids.begin();
  }

  Imu::Ptr mapping_imu_ptr = sensor_manager.getSensorPtr<Imu>(mapping_imu_id);
  CHECK(mapping_imu_ptr);
  return mapping_imu_ptr;
}

Lidar::Ptr getSelectedLidar(const SensorManager& sensor_manager) {
  aslam::SensorIdSet all_lidar_ids;
  sensor_manager.getAllSensorIdsOfType(SensorType::kLidar, &all_lidar_ids);

  if (all_lidar_ids.empty()) {
    VLOG(3) << "No Lidars found in sensor manager!";
    return Lidar::Ptr();
  }

  aslam::SensorId mapping_lidar_id;
  if (all_lidar_ids.size() > 1u) {
    CHECK(
        !FLAGS_selected_lidar_sensor_id.empty() &&
        mapping_lidar_id.fromHexString(FLAGS_selected_lidar_sensor_id))
        << "If more than one Lidar is provided in the sensor manager, use "
        << "--selected_lidar_sensor_id to select which one to use.";

    CHECK(
        sensor_manager.hasSensor(mapping_lidar_id) &&
        (sensor_manager.getSensorType(mapping_lidar_id) == SensorType::kLidar))
        << "The sensor id provided by --selected_lidar_sensor_id is not "
        << "in the sensor manager or is not a Lidar!";
  } else {
    mapping_lidar_id = *all_lidar_ids.begin();
  }

  Lidar::Ptr mapping_lidar_ptr =
      sensor_manager.getSensorPtr<Lidar>(mapping_lidar_id);
  CHECK(mapping_lidar_ptr);
  return mapping_lidar_ptr;
}

Odometry6DoF::Ptr getSelectedOdometry6DoFSensor(
    const SensorManager& sensor_manager) {
  aslam::SensorIdSet all_odometry_6dof_ids;
  sensor_manager.getAllSensorIdsOfType(
      SensorType::kOdometry6DoF, &all_odometry_6dof_ids);

  if (all_odometry_6dof_ids.empty()) {
    VLOG(3) << "No Odometry6DoF sensors found in sensor manager!";
    return Odometry6DoF::Ptr();
  }

  aslam::SensorId mapping_odometry_6dof_id;
  if (all_odometry_6dof_ids.size() > 1u) {
    CHECK(
        !FLAGS_selected_odometry_6dof_sensor_id.empty() &&
        mapping_odometry_6dof_id.fromHexString(
            FLAGS_selected_odometry_6dof_sensor_id))
        << "If more than one Odometry6DoF sensor is provided in the "
        << "sensor manager, use --selected_odometry_6dof_sensor_id to select "
        << "which one to use.";

    CHECK(
        sensor_manager.hasSensor(mapping_odometry_6dof_id) &&
        (sensor_manager.getSensorType(mapping_odometry_6dof_id) ==
         SensorType::kOdometry6DoF))
        << "The sensor id provided by --selected_odometry_6dof_sensor_id is "
        << "not in the sensor manager or is not a Odometry6DoF sensor!";
  } else {
    mapping_odometry_6dof_id = *all_odometry_6dof_ids.begin();
  }

  Odometry6DoF::Ptr mapping_odometry_6dof_ptr =
      sensor_manager.getSensorPtr<Odometry6DoF>(mapping_odometry_6dof_id);
  CHECK(mapping_odometry_6dof_ptr);
  return mapping_odometry_6dof_ptr;
}

LoopClosureSensor::Ptr getSelectedLoopClosureSensor(
    const SensorManager& sensor_manager) {
  aslam::SensorIdSet all_loop_closure_ids;
  sensor_manager.getAllSensorIdsOfType(
      SensorType::kLoopClosureSensor, &all_loop_closure_ids);

  if (all_loop_closure_ids.empty()) {
    LOG(WARNING) << "No LoopClosure sensors found in sensor manager!";

    return LoopClosureSensor::Ptr();
  }

  aslam::SensorId mapping_loop_closure_id;
  if (all_loop_closure_ids.size() > 1u) {
    CHECK(
        !FLAGS_selected_loop_closure_sensor_id.empty() &&
        mapping_loop_closure_id.fromHexString(
            FLAGS_selected_loop_closure_sensor_id))
        << "If more than one LoopClosure sensor is provided in the "
        << "sensor manager, use --selected_loop_closure_sensor_id to select "
        << "which one to use.";

    CHECK(
        sensor_manager.hasSensor(mapping_loop_closure_id) &&
        (sensor_manager.getSensorType(mapping_loop_closure_id) ==
         SensorType::kLoopClosureSensor))
        << "The sensor id provided by --selected_loop_closure_sensor_id is "
        << "not in the sensor manager or is not a LoopClosure sensor!";
  } else {
    mapping_loop_closure_id = *all_loop_closure_ids.begin();
  }

  LoopClosureSensor::Ptr mapping_loop_closure_ptr =
      sensor_manager.getSensorPtr<LoopClosureSensor>(mapping_loop_closure_id);
  CHECK(mapping_loop_closure_ptr);
  return mapping_loop_closure_ptr;
}

WheelOdometry::Ptr getSelectedWheelOdometrySensor(
    const SensorManager& sensor_manager) {
  aslam::SensorIdSet all_wheel_odometry_ids;
  sensor_manager.getAllSensorIdsOfType(
      SensorType::kWheelOdometry, &all_wheel_odometry_ids);

  if (all_wheel_odometry_ids.empty()) {
    LOG(WARNING) << "No WheelOdometry sensors found in sensor manager!";
    return WheelOdometry::Ptr();
  }

  aslam::SensorId mapping_wheel_odometry_id;
  if (all_wheel_odometry_ids.size() > 1u) {
    CHECK(
        !FLAGS_selected_wheel_odometry_sensor_id.empty() &&
        mapping_wheel_odometry_id.fromHexString(
            FLAGS_selected_wheel_odometry_sensor_id))
        << "If more than one WheelOdometry sensor is provided in the "
        << "sensor manager, use --selected_wheel_odometry_sensor_id to select "
        << "which one to use.";

    CHECK(
        sensor_manager.hasSensor(mapping_wheel_odometry_id) &&
        (sensor_manager.getSensorType(mapping_wheel_odometry_id) ==
         SensorType::kWheelOdometry))
        << "The sensor id provided by --selected_wheel_odometry_sensor_id is "
        << "not in the sensor manager or is not a WheelOdometry sensor!";
  } else {
    mapping_wheel_odometry_id = *all_wheel_odometry_ids.begin();
  }

  WheelOdometry::Ptr mapping_wheel_odometry_ptr =
      sensor_manager.getSensorPtr<WheelOdometry>(mapping_wheel_odometry_id);
  CHECK(mapping_wheel_odometry_ptr);
  return mapping_wheel_odometry_ptr;
}

GpsUtm::Ptr getSelectedGpsUtmSensor(const SensorManager& sensor_manager) {
  aslam::SensorIdSet all_gps_utm_ids;
  sensor_manager.getAllSensorIdsOfType(SensorType::kGpsUtm, &all_gps_utm_ids);

  if (all_gps_utm_ids.empty()) {
    VLOG(3) << "No GpsUtm sensors found in sensor manager!";
    return GpsUtm::Ptr();
  }

  aslam::SensorId mapping_gps_utm_id;
  if (all_gps_utm_ids.size() > 1u) {
    CHECK(
        !FLAGS_selected_gps_utm_sensor_id.empty() &&
        mapping_gps_utm_id.fromHexString(FLAGS_selected_gps_utm_sensor_id))
        << "If more than one GpsUtm sensor is provided in the "
        << "sensor manager, use --selected_gps_utm_sensor_id to select "
        << "which one to use.";

    CHECK(
        sensor_manager.hasSensor(mapping_gps_utm_id) &&
        (sensor_manager.getSensorType(mapping_gps_utm_id) ==
         SensorType::kGpsUtm))
        << "The sensor id provided by --selected_gps_utm_sensor_id is "
        << "not in the sensor manager or is not a GpsUtm sensor!";
  } else {
    mapping_gps_utm_id = *all_gps_utm_ids.begin();
  }

  GpsUtm::Ptr mapping_gps_utm_ptr =
      sensor_manager.getSensorPtr<GpsUtm>(mapping_gps_utm_id);
  CHECK(mapping_gps_utm_ptr);
  return mapping_gps_utm_ptr;
}

GpsWgs::Ptr getSelectedGpsWgsSensor(const SensorManager& sensor_manager) {
  aslam::SensorIdSet all_gps_utm_ids;
  sensor_manager.getAllSensorIdsOfType(SensorType::kGpsWgs, &all_gps_utm_ids);

  if (all_gps_utm_ids.empty()) {
    VLOG(3) << "No GpsWgs sensors found in sensor manager!";
    return GpsWgs::Ptr();
  }

  aslam::SensorId mapping_gps_utm_id;
  if (all_gps_utm_ids.size() > 1u) {
    CHECK(
        !FLAGS_selected_gps_utm_sensor_id.empty() &&
        mapping_gps_utm_id.fromHexString(FLAGS_selected_gps_utm_sensor_id))
        << "If more than one GpsWgs sensor is provided in the "
        << "sensor manager, use --selected_gps_utm_sensor_id to select "
        << "which one to use.";

    CHECK(
        sensor_manager.hasSensor(mapping_gps_utm_id) &&
        (sensor_manager.getSensorType(mapping_gps_utm_id) ==
         SensorType::kGpsWgs))
        << "The sensor id provided by --selected_gps_utm_sensor_id is "
        << "not in the sensor manager or is not a GpsWgs sensor!";
  } else {
    mapping_gps_utm_id = *all_gps_utm_ids.begin();
  }

  GpsWgs::Ptr mapping_gps_utm_ptr =
      sensor_manager.getSensorPtr<GpsWgs>(mapping_gps_utm_id);
  CHECK(mapping_gps_utm_ptr);
  return mapping_gps_utm_ptr;
}

Absolute6DoF::Ptr getSelectedAbsolute6DoFSensor(
    const SensorManager& sensor_manager) {
  aslam::SensorIdSet all_absolute_6dof_ids;
  sensor_manager.getAllSensorIdsOfType(
      SensorType::kAbsolute6DoF, &all_absolute_6dof_ids);

  if (all_absolute_6dof_ids.empty()) {
    VLOG(3) << "No Absolute6DoF sensors found in sensor manager!";
    return Absolute6DoF::Ptr();
  }

  aslam::SensorId mapping_absolute_6dof_id;
  if (all_absolute_6dof_ids.size() > 1u) {
    CHECK(
        !FLAGS_selected_absolute_6dof_sensor_id.empty() &&
        mapping_absolute_6dof_id.fromHexString(
            FLAGS_selected_absolute_6dof_sensor_id))
        << "If more than one Absolute6DoF sensor is provided in the "
        << "sensor manager, use --selected_absolute_6dof_sensor_id to select "
        << "which one to use.";

    CHECK(
        sensor_manager.hasSensor(mapping_absolute_6dof_id) &&
        (sensor_manager.getSensorType(mapping_absolute_6dof_id) ==
         SensorType::kAbsolute6DoF))
        << "The sensor id provided by --selected_absolute_6dof_sensor_id is "
        << "not in the sensor manager or is not a Absolute6DoF sensor!";
  } else {
    mapping_absolute_6dof_id = *all_absolute_6dof_ids.begin();
  }

  Absolute6DoF::Ptr mapping_absolute_6dof_ptr =
      sensor_manager.getSensorPtr<Absolute6DoF>(mapping_absolute_6dof_id);
  CHECK(mapping_absolute_6dof_ptr);
  return mapping_absolute_6dof_ptr;
}

PointCloudMapSensor::Ptr getSelectedPointCloudMapSensor(
    const SensorManager& sensor_manager) {
  aslam::SensorIdSet all_point_cloud_map_sensor_ids;
  sensor_manager.getAllSensorIdsOfType(
      SensorType::kPointCloudMapSensor, &all_point_cloud_map_sensor_ids);

  if (all_point_cloud_map_sensor_ids.empty()) {
    VLOG(3) << "No point cloud map sensors found in sensor manager!";
    return PointCloudMapSensor::Ptr();
  }

  aslam::SensorId point_cloud_map_sensor_id;
  if (all_point_cloud_map_sensor_ids.size() > 1u) {
    CHECK(
        !FLAGS_selected_point_cloud_map_sensor_id.empty() &&
        point_cloud_map_sensor_id.fromHexString(
            FLAGS_selected_point_cloud_map_sensor_id))
        << "If more than one point cloud map sensor is provided in the "
        << "sensor manager, use --selected_point_cloud_map_sensor_id to select "
        << "which one to use.";

    CHECK(
        sensor_manager.hasSensor(point_cloud_map_sensor_id) &&
        (sensor_manager.getSensorType(point_cloud_map_sensor_id) ==
         SensorType::kPointCloudMapSensor))
        << "The sensor id provided by --selected_point_cloud_map_sensor_id is "
        << "not in the sensor manager or is not a point cloud map sensor!";
  } else {
    point_cloud_map_sensor_id = *all_point_cloud_map_sensor_ids.begin();
  }

  PointCloudMapSensor::Ptr point_cloud_map_sensor =
      sensor_manager.getSensorPtr<PointCloudMapSensor>(
          point_cloud_map_sensor_id);
  CHECK(point_cloud_map_sensor);
  return point_cloud_map_sensor;
}

}  // namespace vi_map
