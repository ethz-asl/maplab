cmake_minimum_required(VERSION 2.8.3)
project(registration_toolbox)

include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++14" COMPILER_SUPPORTS_CXX14)
if(COMPILER_SUPPORTS_CXX14)
  set(CMAKE_CXX_STANDARD 14)
  set(CMAKE_CXX_FLAGS         "-Wall -Wextra -msse -msse2 -msse3 -msse4 -O3 -DNDEBUG")
  set(CMAKE_CXX_FLAGS_DEBUG   "-O0 -g")
  set(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG")
else()
  message(FATAL_ERROR "The compiler does not support C++14.")
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

#############
# LIBRARIES #
#############

cs_add_library(${PROJECT_NAME}
  src/alignment/lpm-aligment.cc
  src/common/common.cc
  src/common/registration-factory.cc
  src/common/registration-gflags.cc
  src/common/supported.cc
  src/model/registration-result.cc
  src/lpm-icp-controller.cc
  src/mock-controller.cc
  src/pcl-gicp-controller.cc
  src/pcl-icp-controller.cc
)

##########
# TESTS #
##########
cs_add_executable(${PROJECT_NAME}_driver test/aligner-driver.cc)
target_link_libraries(${PROJECT_NAME}_driver ${PROJECT_NAME})
catkin_add_gtest(test_registration_toolbox test/test-regbox.cc)
target_link_libraries(test_registration_toolbox ${PROJECT_NAME})

##########
# EXPORT #
##########
cs_install()
cs_export()
