#!/bin/bash
export PATH=/usr/local/bin/:$PATH

###############################################################################
# Parse scripts input arguments.
###############################################################################
JOB=""
JOB_DIRECTORY=""
RESULT_DIRECTORY=""
CATKIN_WORKSPACE="${WORKSPACE}"
DATASET_PATH_PREFIX=""

function print_args_help {
    echo "Usage: swe-automatic-evaluation-ci.sh "
    echo "  [{-j|--job_list}=jobyamls]"
    echo "  [{-d|--job_dir} directory containing jobs]"
    echo "  [{-r|--result_dir} skip gtest execution]"
    echo "  [{-w|--workspace} path to catkin workspace that contains the swe build]"   
    echo "  [{-p|--dataset_path_prefix} path prefix for datasets]"
}
           
for i in "$@"
do
case $i in
  -j=*|--job_list=*)
  JOB="${i#*=}"
  ;;
  -d=*|--job_dir=*)
  JOB_DIRECTORY="${i#*=}"
  ;;
  -r=*|--result_dir=*)
  RESULT_DIRECTORY="${i#*=}"
  ;;
  -w=*|--workspace=*)
  CATKIN_WORKSPACE="${i#*=}"
  ;;
  -p=*|--dataset_path_prefix=*)
  DATASET_PATH_PREFIX="${i#*=}"
  ;;
  *)
  print_args_help
  exit 0
  ;;
esac
done

if [ ! -d "$CATKIN_WORKSPACE" ]; then
    echo "Could not find the specified catkin workspace: \"$CATKIN_WORKSPACE\""
    exit -1
fi

if [ ! -d "$RESULT_DIRECTORY" ]; then
    echo "Could not find the specified result directory: \"$RESULT_DIRECTORY\""
    exit -1
fi

if [ ! -d "$DATASET_PATH_PREFIX" ]; then
    echo "Could not find the specified dataset path prefix: \"$DATASET_PATH_PREFIX\""
    exit -1
fi

###############################################################################
# Generate the list of jobs to run.
###############################################################################
shopt -s nullglob
JOBS_FROM_DIRECTORY=()
if [ -d "$JOB_DIRECTORY" ]; then
  ALL_JOBS_FROM_DIRECTORY=($JOB_DIRECTORY/*.yaml)
  
  # Remove jobs that start with the keyword DISABLED_.
  for job in "${ALL_JOBS_FROM_DIRECTORY[@]}"
  do
    if [[ $(basename $job) != DISABLED_* ]]; then
      JOBS_FROM_DIRECTORY=("${JOBS_FROM_DIRECTORY[@]}" "$job")
    fi
  done
fi   

if [ "${#JOBS_FROM_DIRECTORY[@]}" -gt 0 ]; then
    JOBS_TO_RUN=("${JOBS_TO_RUN[@]}" "${JOBS_FROM_DIRECTORY[@]}")
fi

if [ -n "$JOB" ]; then
    JOBS_TO_RUN=("${JOBS_TO_RUN[@]}" "$JOB")
fi

if [ "${#JOBS_TO_RUN[@]}" -eq 0 ]; then
    echo "No jobs defined to run. Exiting!"
    exit -1
fi

###############################################################################
# Setup the sliding-window-estimator evaluation environment.
###############################################################################
source $CATKIN_WORKSPACE/devel/setup.bash
EVALUATION_SCRIPT="$(find $(catkin_find swe_pipeline | grep src) | grep run_evaluation.py)"

echo
echo "Automatic evaluation setup:"
echo "==========================="
echo "Catkin workspace: $CATKIN_WORKSPACE"
echo "Dataset path prefix: $DATASET_PATH_PREFIX"
echo "Jobs to run: "
printf ' - %s\n' "${JOBS_TO_RUN[@]}"
echo "Location of evaluation script: $EVALUATION_SCRIPT"
echo "Result directory: $RESULT_DIRECTORY"
echo "-------------------------------------"
echo

###############################################################################
# Run all test jobs and upload the results.
###############################################################################
TIMESTRING=$(date +"%Y-%m-%d_%H-%M-%S")

for job in "${JOBS_TO_RUN[@]}"
do
  # Run the job.
  job_output_dir="$RESULT_DIRECTORY/$TIMESTRING-$(basename $job)"

  echo "Running job \"$job\" and writing output to \"$job_output_dir\":"
  echo "------------------------------------------------"

  python $EVALUATION_SCRIPT --job $job                           \
                            --output-directory $job_output_dir   \
                            --dataset_path $DATASET_PATH_PREFIX
  RETVAL=$?
  if [ $RETVAL -ne 0 ]; then
   exit $RETVAL
  fi

  # Move the map to the output folder.
  cp -R map/ $job_output_dir
  rm -rf map/

  # Store the git hash that produced this result.
  touch $job_output_dir/evaluated-git-revision
  echo "$(cd $(dirname $EVALUATION_SCRIPT); git rev-parse HEAD)" > $job_output_dir/evaluated-git-revision
done

exit 0
