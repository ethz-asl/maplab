#!/bin/bash

if [ "$1" != "" ]; then
  ROSBAGS=$(ls ${1}*.bag)
  OPTIONS="${*:2}"

  echo "Playing multiple bags:"
  echo " - rosbag folder:       $1"
  echo " - rosbag play options: $OPTIONS"

  echo "Command: rosbag play ${ROSBAGS} ${OPTIONS}"
  rosbag play --clock ${ROSBAGS} ${OPTIONS} /tf:=/tf_dev_null /tf_static:=/tf_static_dev_null

else
  echo "Usage: play_multibag.sh [rosbags folder] [rosbag play options ...]"
fi
