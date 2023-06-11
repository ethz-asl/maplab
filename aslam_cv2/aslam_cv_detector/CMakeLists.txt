cmake_minimum_required(VERSION 2.8.3)
project(aslam_cv_detector)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

#############
# LIBRARIES #
#############
add_definitions(-std=c++11)

cs_add_library(${PROJECT_NAME}_kaze   
  src/kaze/fed.cpp
  src/kaze/KAZE.cpp
  src/kaze/nldiffusion_functions.cpp
  src/kaze/utils.cpp
)

cs_add_library(${PROJECT_NAME}_lsd 
  src/lsd/lsd-opencv.cc
)

cs_add_library(${PROJECT_NAME} 
  src/line-segment-detector.cc
)

target_link_libraries(${PROJECT_NAME}
  ${PROJECT_NAME}_kaze 
  ${PROJECT_NAME}_lsd
)

##########
# EXPORT #
##########
cs_install()
cs_export()
