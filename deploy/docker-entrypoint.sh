#!/bin/bash
set -e

source "/opt/ros/$ROS_DISTRO/setup.bash" --
if [ -f "/maplab/devel/setup.bash" ]; then
  source "/maplab/devel/setup.bash"
fi

export PATH="/usr/lib/ccache:$PATH"
export CCACHE_DIR="/maplab/.ccache"

exec "$@"
