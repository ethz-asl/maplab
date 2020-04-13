#include "maplab-node/datasource.h"

DEFINE_int64(
    imu_to_camera_time_offset_ns, 0,
    "Fixed time offset of IMU to the camera, such that: t_imu - offset = "
    "t_cam");

DEFINE_double(
    maplab_throttle_frequency_odometry, 20,
    "Maximum frequency of odometry measurements that are passed on to the "
    "maplab_node.");

DEFINE_double(
    maplab_batch_imu_measurements_at_frequency, 20,
    "Individual IMU measurements are collected and forwarded to the rest of "
    "the system in batches, rather than individually at a high frequency. If "
    "set to <= 0, batching is disabled.");
