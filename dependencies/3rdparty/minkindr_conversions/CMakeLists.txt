cmake_minimum_required(VERSION 2.8.3)
project(minkindr_conversions)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

add_definitions(-std=c++11)

#############
# LIBRARIES #
#############

##########
# GTESTS #
##########
catkin_add_gtest(kindr_tf_test
  test/kindr_tf_test.cc
)
target_link_libraries(kindr_tf_test ${catkin_LIBRARIES})

catkin_add_gtest(kindr_msg_test
  test/kindr_msg_test.cc
)
target_link_libraries(kindr_msg_test ${catkin_LIBRARIES})

##########
# EXPORT #
##########
cs_install()
cs_export()
