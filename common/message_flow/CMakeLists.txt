cmake_minimum_required(VERSION 2.8.3)
project(message_flow)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

###########
# LIBRARY #
###########
add_definitions(--std=c++11)
cs_add_library(${PROJECT_NAME} 
  src/message-flow.cc
)

#########
# TESTS #
#########
catkin_add_gtest(test_message_flow test/test-message-flow.cc)
target_link_libraries(test_message_flow ${PROJECT_NAME})

##########
# EXPORT #
##########
cs_install()
cs_export()
