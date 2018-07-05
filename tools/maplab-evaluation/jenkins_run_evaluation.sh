#!/bin/bash
set -e

source $WORKSPACE/devel/setup.bash

# Copy-paste from run_build.
START_ROSCORE=true
function kill_roscore_on_exit {
  if $START_ROSCORE && [[ "$ROS_PID" -gt 0 ]] ; then
    # Kill roscore.
    kill $ROS_PID
  fi
}

if $START_ROSCORE ; then
  ROS_PORT=-1
  ROS_HOME=$HOME/.ros

  # Check if lsof is installed.
  if ! (command -v lsof > /dev/null) ; then
    if [[ $(uname) == 'Linux' ]] ; then
      export DEBIAN_FRONTEND=noninteractive
      sudo apt-get install -y lsof
    else
      fatal "lsof not installed: can't scan for free port for the roscore."
    fi
  fi

  # Check for a free port.
  echo "Looking for an unused port for the roscore."
  for i in `seq 12000 13000`
  do
    # Check if port is unused.
    if ! (lsof -i :$i > /dev/null ) ; then
      export ROS_PORT=$i
      break
    fi
  done

  if [ $ROS_PORT -lt 0 ] ; then
    fatal "Couldn't find an unused port for the roscore."
  fi

  # Start roscore.
  export ROS_MASTER_URI="http://localhost:$ROS_PORT"
  echo "Starting roscore on port $ROS_PORT."
  roscore -p $ROS_PORT > /dev/null &
  ROS_PID=$!
  trap kill_roscore_on_exit EXIT
fi
# End copy-paste.

rosrun maplab_evaluation perform_evaluation.py \
  --results_output_folder=$WORKSPACE/results \
  --disable_progress_bars \
  $WORKSPACE/src/tools/maplab-evaluation/evaluation_data/experiments
rosrun maplab_evaluation long_term_report_generator.py \
  --results_folders "$WORKSPACE/results" \
      "/var/www/public_artifacts/${JOB_NAME}/" \
  --max_number_of_results_to_use=10 \
  --output_folder="$WORKSPACE/results"
