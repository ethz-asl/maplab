cmake_minimum_required(VERSION 2.8.3)
project(aslam_cv_pipeline)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

#############
# LIBRARIES #
#############
set(HEADERS
  include/aslam/pipeline/visual-npipeline.h
  include/aslam/pipeline/visual-pipeline.h
  include/aslam/pipeline/visual-pipeline-null.h
)

set(SOURCES
  src/visual-npipeline.cc
  src/visual-pipeline-null.cc
  src/visual-pipeline.cc
)

cs_add_library(${PROJECT_NAME} ${SOURCES} ${HEADERS})

SET(CMAKE_SHARED_LIBRARY_LINK_CXX_FLAGS "${CMAKE_SHARED_LIBRARY_LINK_CXX_FLAGS} -lpthread")

##########
# GTESTS #
##########
catkin_add_gtest(test_visual-npipeline test/test-visual-npipeline.cc)
target_link_libraries(test_visual-npipeline ${PROJECT_NAME})

##########
# EXPORT #
##########
cs_install()
cs_export()
