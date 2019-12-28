#!/usr/bin/env bash

VI_MAP=$1
ROSBAG=$2
TOPIC=$3
SENSOR_CALIBRATION_FILE=$4
VI_MAP_OUT=$5

rosrun resource_importer resource_importer \
  --map_path $VI_MAP \
  --map_output_path $VI_MAP_OUT \
  --rosbag_path $ROSBAG \
  --resource_topic $TOPIC \
  --v=1 \
  --alsologtostderr \
  --sensor_calibration_file=$SENSOR_CALIBRATION_FILE $@
