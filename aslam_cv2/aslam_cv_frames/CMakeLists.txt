cmake_minimum_required(VERSION 2.8.3)
project(aslam_cv_frames)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

#############
# LIBRARIES #
#############
set(SOURCES
  src/visual-frame.cc
  src/visual-nframe.cc
)
cs_add_library(${PROJECT_NAME} ${SOURCES})

add_doxygen(NOT_AUTOMATIC)

##########
# GTESTS #
##########
catkin_add_gtest(test_visual-frame test/test-visual-frame.cc)
target_link_libraries(test_visual-frame ${PROJECT_NAME})

catkin_add_gtest(test_visual-nframe test/test-visual-nframe.cc)
target_link_libraries(test_visual-nframe ${PROJECT_NAME})

##########
# EXPORT #
##########
cs_install()
cs_export()
