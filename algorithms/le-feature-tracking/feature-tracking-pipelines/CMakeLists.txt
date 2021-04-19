cmake_minimum_required(VERSION 2.8.3)
project(feature_tracking_pipelines)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)
add_compile_options(-w)

#############
# LIBRARIES #
#############
cs_add_library(${PROJECT_NAME}
  src/feature-describer-brisk.cc
  src/feature-describer-freak.cc
  src/feature-describer-sift.cc
  src/feature-describer-surf.cc
  src/feature-detector-brisk.cc
  src/feature-detector-gftt.cc
  src/feature-detector-orb.cc
  src/feature-detector-sift.cc
  src/feature-detector-surf.cc
  src/feature-tracker-gyro-aided.cc
  src/feature-tracker-gyro-aided-laser.cc
  src/flags.cc
  src/keyframe-features.cc
  src/helpers.cc
  src/pnp-ransac.cc
)

cs_install()
cs_export()
