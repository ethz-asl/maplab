#ifndef MAPLAB_NODE_FLOW_TOPICS_H_
#define MAPLAB_NODE_FLOW_TOPICS_H_
#include <maplab-common/localization-result.h>
#include <message-flow/message-topic-registration.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <sensors/absolute-6dof-pose.h>
#include <sensors/lidar.h>
#include <sensors/odometry-6dof-pose.h>
#include <sensors/pointcloud-map-sensor.h>
#include <sensors/wheel-odometry-sensor.h>

#include <vi-map/vi-map.h>
#include <vio-common/map-update.h>
#include <vio-common/vio-types.h>
#include <vio-common/vio-update.h>
#include "maplab-node/odometry-estimate.h"
#include "maplab-node/vi-map-with-mutex.h"

// Image data.
MESSAGE_FLOW_TOPIC(IMAGE_MEASUREMENTS, vio::ImageMeasurement::ConstPtr);

// Synchronized images from multiple cameras
MESSAGE_FLOW_TOPIC(SYNCED_NFRAMES, vio::SynchronizedNFrame::Ptr);
// Same as synchronized nframes but also includes feature tracks.
MESSAGE_FLOW_TOPIC(TRACKED_NFRAMES, vio::SynchronizedNFrame::ConstPtr);

// Imu data.
MESSAGE_FLOW_TOPIC(IMU_MEASUREMENTS, vio::BatchedImuMeasurements::ConstPtr);

// Odometry input
MESSAGE_FLOW_TOPIC(ODOMETRY_ESTIMATES, maplab::OdometryEstimate::ConstPtr);

// Lidar scans or other depth sensors.
MESSAGE_FLOW_TOPIC(LIDAR_MEASUREMENTS, vi_map::RosLidarMeasurement::ConstPtr);
MESSAGE_FLOW_TOPIC(
    SYNCED_LIDAR_MEASUREMENTS, vi_map::RosLidarMeasurement::ConstPtr);

// LoopClosure constraints input. Used to add loop closure edges in the map
// from an external source.
MESSAGE_FLOW_TOPIC(
    LOOP_CLOSURE_CONSTRAINTS, vi_map::LoopClosureMeasurement::ConstPtr);
MESSAGE_FLOW_TOPIC(
    SYNCED_LOOP_CLOSURE, vi_map::LoopClosureMeasurement::ConstPtr);

// Absolute6DoF constraints input. Used to add global constraints in the map
// from an external source.
MESSAGE_FLOW_TOPIC(
    ABSOLUTE_6DOF_CONSTRAINTS, vi_map::Absolute6DoFMeasurement::Ptr);
MESSAGE_FLOW_TOPIC(SYNCED_ABSOLUTE_6DOF, vi_map::Absolute6DoFMeasurement::Ptr);

// WheelOdometry constraints input. Used to add relative constraints in the map
// from an external source (e.g. wheel odometry)
MESSAGE_FLOW_TOPIC(
    WHEEL_ODOMETRY_CONSTRAINTS, vi_map::WheelOdometryMeasurement::ConstPtr);
MESSAGE_FLOW_TOPIC(
    SYNCED_WHEEL_ODOMETRY, vi_map::WheelOdometryMeasurement::ConstPtr);

// External point cloud map input. Used to attach accumulated local point cloud
// maps or 3D reconstructions to the pose graph.
MESSAGE_FLOW_TOPIC(
    POINTCLOUD_MAP, vi_map::RosPointCloudMapSensorMeasurement::ConstPtr);
MESSAGE_FLOW_TOPIC(
    SYNCED_POINTCLOUD_MAP, vi_map::RosPointCloudMapSensorMeasurement::ConstPtr);

// External features input. Used to attach externally detected features to
// an existing camera inside an ncamera.
MESSAGE_FLOW_TOPIC(
    EXTERNAL_FEATURES, vi_map::ExternalFeaturesMeasurement::ConstPtr);
MESSAGE_FLOW_TOPIC(
    SYNCED_EXTERNAL_FEATURES, vi_map::ExternalFeaturesMeasurement::ConstPtr);

// Output of the localizer.
MESSAGE_FLOW_TOPIC(LOCALIZATION_RESULT, common::LocalizationResult::ConstPtr);

// Output of the localization handler, fused localization results from all
// sources.
MESSAGE_FLOW_TOPIC(
    FUSED_LOCALIZATION_RESULT, common::LocalizationResult::ConstPtr);

// Raw estimate of the VINS.
MESSAGE_FLOW_TOPIC(MAP_UPDATES, vio::MapUpdate::ConstPtr);

// Resulting map.
MESSAGE_FLOW_TOPIC(RAW_VIMAP, maplab::VIMapWithMutex::ConstPtr);

namespace maplab {
constexpr int kExclusivityGroupIdRawSensorDataSubscribers = 0;
}

#endif  // MAPLAB_NODE_FLOW_TOPICS_H_
