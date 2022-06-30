#!/usr/bin/env bash

. /opt/ros/melodic/setup.bash
. /usr/home/ws/devel/setup.bash
roscore > /dev/null &
exec "$@"
