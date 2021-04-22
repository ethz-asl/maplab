cmake_minimum_required(VERSION 2.8.3)
project(maplab_server_node)

set(CMAKE_CXX_STANDARD 14)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

############
# PROTOBUF #
############
set(PROTO_DEFNS proto/maplab-server-node/robot_missions_information.proto)
PROTOBUF_CATKIN_GENERATE_CPP2("proto" PROTO_SRCS PROTO_HDRS ${PROTO_DEFNS})

#############
# LIBRARIES #
#############
cs_add_library(${PROJECT_NAME}_lib
	src/maplab-server-node.cc
  src/maplab-server-ros-node.cc
	${PROTO_SRCS})

#######
# APP #
#######
cs_add_executable(${PROJECT_NAME}
  app/maplab-server-ros-node-app.cc
)
target_link_libraries(${PROJECT_NAME} ${PROJECT_NAME}_lib)

##########
# GTESTS #
##########
catkin_add_gtest(test_maplab_server_node test/test-maplab-server-node.cc)
target_link_libraries(test_maplab_server_node ${PROJECT_NAME}_lib)
maplab_import_test_maps(test_maplab_server_node)

############
## EXPORT ##
############
cs_install()
cs_export()
