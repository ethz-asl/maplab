#ifndef VI_MAP_SENSOR_UTILS_H_
#define VI_MAP_SENSOR_UTILS_H_

#include <memory>

#include <aslam/common/sensor.h>
#include <sensors/absolute-6dof-pose.h>
#include <sensors/gps-utm.h>
#include <sensors/gps-wgs.h>
#include <sensors/imu.h>
#include <sensors/lidar.h>
#include <sensors/loop-closure-sensor.h>
#include <sensors/odometry-6dof-pose.h>
#include <sensors/pointcloud-map-sensor.h>
#include <sensors/sensor-types.h>
#include <sensors/wheel-odometry-sensor.h>

#include "vi-map/sensor-manager.h"

DECLARE_string(selected_ncamera_sensor_id);
DECLARE_string(selected_imu_sensor_id);
DECLARE_string(selected_lidar_sensor_id);
DECLARE_string(selected_odometry_6dof_sensor_id);
DECLARE_string(selected_loop_closure_sensor_id);
DECLARE_string(selected_absolute_6dof_sensor_id);
DECLARE_string(selected_wheel_odometry_sensor_id);
DECLARE_string(selected_gps_utm_sensor_id);
DECLARE_string(selected_gps_wgs_sensor_id);
DECLARE_string(selected_point_cloud_map_sensor_id);

namespace vi_map {

// These functions try to find an NCamera/Imu/Lidar in the sensor manager, if
// there are multiple such sensors, it will use the
// --ncamera/imu/lidar_sensor_id_for_mapping flags to select the correct one.
// Will return an empty SharedPtr if no sensor was found and fail if multiple
// are found but none was selected by gflag.
// NOTE: These functions could be unified with a macro/function, but
// since they contain a lot of checks and sensor-specific error
// messages/handling the code duplication is necessary to make it easier to find
// and understand the errors.

aslam::NCamera::Ptr getSelectedNCamera(const SensorManager& sensor_manager);

Imu::Ptr getSelectedImu(const SensorManager& sensor_manager);

Lidar::Ptr getSelectedLidar(const SensorManager& sensor_manager);

Odometry6DoF::Ptr getSelectedOdometry6DoFSensor(
    const SensorManager& sensor_manager);

LoopClosureSensor::Ptr getSelectedLoopClosureSensor(
    const SensorManager& sensor_manager);

Absolute6DoF::Ptr getSelectedAbsolute6DoFSensor(
    const SensorManager& sensor_manager);

WheelOdometry::Ptr getSelectedWheelOdometrySensor(
    const SensorManager& sensor_manager);

PointCloudMapSensor::Ptr getSelectedPointCloudMapSensor(
    const SensorManager& sensor_manager);

GpsUtm::Ptr getSelectedGpsUtmSensor(const SensorManager& sensor_manager);

GpsWgs::Ptr getSelectedGpsWgsSensor(const SensorManager& sensor_manager);

}  // namespace vi_map

#endif  // VI_MAP_SENSOR_UTILS_H_
