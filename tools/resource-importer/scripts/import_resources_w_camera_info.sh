#!/usr/bin/env bash

VI_MAP=$1
ROSBAG=$2
TOPIC=$3
CAMERA_INFO_TOPIC=$4
VI_MAP_OUT=$5

rosrun resource_importer resource_importer \
  --map_path $VI_MAP \
  --map_output_path $VI_MAP_OUT \
  --rosbag_path $ROSBAG \
  --resource_topic $TOPIC \
  --v=1 \
  --alsologtostderr \
  --camera_calibration_topic $CAMERA_INFO_TOPIC \
  --camera_extrinsics_imu_frame="imu" \
  --camera_extrinsics_camera_frame="depth"
