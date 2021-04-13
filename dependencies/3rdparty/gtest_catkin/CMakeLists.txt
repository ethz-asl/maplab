cmake_minimum_required(VERSION 2.8.3)
project(gtest_catkin)

find_package(catkin_simple REQUIRED)
catkin_simple()

include(ExternalProject)

file(MAKE_DIRECTORY ${CATKIN_DEVEL_PREFIX}/include)

set(VERSION 1.7.0)
ExternalProject_Add(gtest_src
  URL https://github.com/google/googletest/archive/release-${VERSION}.zip
  UPDATE_COMMAND ""
  INSTALL_COMMAND cp libgtest* ${CATKIN_DEVEL_PREFIX}/lib/
)

install(DIRECTORY ${CATKIN_DEVEL_PREFIX}/include/gtest
        DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.h")
install(DIRECTORY ${CATKIN_DEVEL_PREFIX}/lib/
        DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        FILES_MATCHING PATTERN "libgtest*")
cs_export(INCLUDE_DIRS ${CATKIN_DEVEL_PREFIX}/include
          LIBRARIES gtest)
