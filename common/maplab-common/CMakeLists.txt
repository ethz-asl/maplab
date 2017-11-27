cmake_minimum_required(VERSION 2.8.3)
project(maplab_common)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

include(cmake/maplab_common-extras.cmake.in)

############
# PROTOBUF #
############
set(PROTO_DEFNS proto/maplab-common/eigen.proto
                proto/maplab-common/id.proto)
PROTOBUF_CATKIN_GENERATE_CPP2("proto" PROTO_SRCS PROTO_HDRS ${PROTO_DEFNS})
include_directories(${CMAKE_CURRENT_BINARY_DIR})

find_package(PythonLibs REQUIRED)
include_directories(${PYTHON_INCLUDE_DIRS})

#############
# LIBRARIES #
#############
find_package (Threads)
cs_add_library(${PROJECT_NAME} src/binary-serialization.cc
                               src/breakpoints.cc
                               src/combinatorial.cc
                               src/condition.cc
                               src/cubic-spline.cc
                               src/delayed-notification.cc
                               src/feature-descriptor-ref.cc
                               src/file-lock.cc
                               src/file-logger.cc
                               src/file-system-tools.cc
                               src/geometry.cc
                               src/global-coordinate-tools.cc
                               src/gnuplot-interface.cc
                               src/gravity-provider.cc
                               src/histograms.cc
                               src/multi-threaded-progress-bar.cc
                               src/progress-bar.cc
                               src/proto-serialization-helper.cc
                               src/python-interface.cc
                               src/shared-gflags.cc
                               src/sigint-breaker.cc
                               src/stringprintf.cc
                               src/test/testing-entrypoint.cc
                               src/threading-helpers.cc
                               src/tridiagonal-matrix.cc
                               src/unique-id.cc
                               ${PROTO_SRCS}
                               ${PROTO_HDRS})
target_link_libraries(${PROJECT_NAME} ${CMAKE_THREAD_LIBS_INIT} ${PYTHON_LIBRARIES} readline)

#############
## TESTING ##
#############
SET(PROJECT_TEST_DATA "maplab_test_data")
add_custom_target(${PROJECT_TEST_DATA})
add_custom_command(TARGET ${PROJECT_TEST_DATA}
COMMAND rm -rf "./${PROJECT_TEST_DATA}" && mkdir ./${PROJECT_TEST_DATA}/ && tar -xvzf ${MAPLAB_TEST_DATA_DIR}/maplab_common_test_data/maplab_common_test_data.tar.gz -C ./${PROJECT_TEST_DATA}/)

catkin_add_gtest(test_accessors
  test/test_accessors.cc)
target_link_libraries(test_accessors ${PROJECT_NAME})

catkin_add_gtest(test_fixed_size_queue
    test/test_fixed_size_queue.cc)
target_link_libraries(test_fixed_size_queue ${PROJECT_NAME})

catkin_add_gtest(test_delayed_notification_test
                 test/test_delayed_notification.cc)
target_link_libraries(test_delayed_notification_test ${PROJECT_NAME})

catkin_add_gtest(test_eigen_proto_test
  test/test_eigen_proto_test.cc)
target_link_libraries(test_eigen_proto_test ${PROJECT_NAME})

catkin_add_gtest(test_file_lock_test
  test/test_file_lock.cc)
target_link_libraries(test_file_lock_test ${PROJECT_NAME})

catkin_add_gtest(test_geometry_test
                 test/test_geometry.cc)
target_link_libraries(test_geometry_test ${PROJECT_NAME})

catkin_add_gtest(test_kruskal_max_span_tree
  test/test_kruskal_max_span_tree.cc)
target_link_libraries(test_kruskal_max_span_tree ${PROJECT_NAME})

catkin_add_gtest(test_multi_threaded_progress_bar
  test/test_multi_threaded_progress_bar.cc)
target_link_libraries(test_multi_threaded_progress_bar ${PROJECT_NAME})

catkin_add_gtest(test_ordered_monitors test/test_ordered_monitors.cc)
target_link_libraries(test_ordered_monitors ${PROJECT_NAME})

catkin_add_gtest(test_parallel_process
  test/test_parallel_process.cc)
target_link_libraries(test_parallel_process ${PROJECT_NAME})

catkin_add_gtest(test_progress_bar
  test/test_progress_bar.cc)
target_link_libraries(test_progress_bar ${PROJECT_NAME})

catkin_add_gtest(test_stl_yamlcpp
  test/test_stl_yaml_serialization.cc)
target_link_libraries(test_stl_yamlcpp ${PROJECT_NAME})

catkin_add_gtest(test_quaternion_math
  test/test-quaternion-math.cc)
target_link_libraries(test_quaternion_math ${PROJECT_NAME})

catkin_add_gtest(test_eigen_yamlcpp
  test/test_eigen_yaml_serialization.cc)
target_link_libraries(test_eigen_yamlcpp ${PROJECT_NAME})

catkin_add_gtest(test_file_system_tools
  test/test_file_system_tools.cc)
target_link_libraries(test_file_system_tools ${PROJECT_NAME})
add_dependencies(test_file_system_tools ${PROJECT_TEST_DATA})

catkin_add_gtest(test_vector_window_operations
  test/test_vector-window-operations.cc)
target_link_libraries(test_vector_window_operations ${PROJECT_NAME})

catkin_add_gtest(test_temporal_buffer test/test_temporal_buffer.cc)
target_link_libraries(test_temporal_buffer ${PROJECT_NAME})

catkin_add_gtest(test_combinatorial
  test/test_combinatorial.cc)
target_link_libraries(test_combinatorial ${PROJECT_NAME})

catkin_add_gtest(test_feature_descriptor_ref
  test/test_feature-descriptor-ref.cc)
target_link_libraries(test_feature_descriptor_ref ${PROJECT_NAME})

catkin_add_gtest(test_global_coordinate_tools
  test/test_global-coordinate-tools.cc)
target_link_libraries(test_global_coordinate_tools ${PROJECT_NAME})

catkin_add_gtest(test_file_logger
  test/test_file_logger.cc)
target_link_libraries(test_file_logger ${PROJECT_NAME})

catkin_add_gtest(test_python_interface
  test/test_python_interface.cc)
target_link_libraries(test_python_interface ${PROJECT_NAME} ${PYTHON_LIBRARIES})

catkin_add_gtest(test_string_tools
  test/test_string_tools.cc)
target_link_libraries(test_string_tools ${PROJECT_NAME})

catkin_add_gtest(test_thread_safe_queue
  test/test_threadsafe_queue.cc)
target_link_libraries(test_thread_safe_queue ${PROJECT_NAME})

catkin_add_gtest(test_threaded_task_queue_processor
  test/test-threaded-task-queue-processor.cc)
target_link_libraries(test_threaded_task_queue_processor ${PROJECT_NAME})

catkin_add_gtest(test_monitor test/test_monitor.cc)
target_link_libraries(test_monitor ${PROJECT_NAME})

catkin_add_gtest(test_timeout_counter
  test/test-timeout-counter.cc)
target_link_libraries(test_timeout_counter ${PROJECT_NAME})

catkin_add_gtest(test_interpolation_helpers
  test/test-interpolation-helpers.cc)
target_link_libraries(test_interpolation_helpers ${PROJECT_NAME})

############
## EXPORT ##
############
cs_install()
cs_export(CFG_EXTRAS maplab_common-extras.cmake)
