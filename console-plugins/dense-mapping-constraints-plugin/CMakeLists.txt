cmake_minimum_required (VERSION 2.8)
project(dense_mapping_constraints_plugin)

find_package(catkin_simple REQUIRED)
find_package(Open3D REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

include_directories(${Open3D_INCLUDE_DIRS})
link_directories(${Open3D_LIBRARY_DIRS})

add_definitions(-fPIC -shared)

cs_add_library(${PROJECT_NAME}
                               src/dense-mapping-alignment.cc
                               src/dense-mapping-common.cc
                               src/dense-mapping-constraints.cc
                               src/dense-mapping-gflags.cc
                               src/dense-mapping-plugin.cc
                               src/dense-mapping-search.cc
                               src/dense-mapping-selection.cc
                               src/dense-mapping.cc
)
create_console_plugin(${PROJECT_NAME})

target_link_libraries(${PROJECT_NAME} ${Open3D_LIBRARIES})

#############
## TESTING ##
#############
catkin_add_gtest(test_dense_mapping test/test-dense-mapping.cc)
target_link_libraries(test_dense_mapping ${PROJECT_NAME})
maplab_import_test_maps(test_dense_mapping)

##################
# CMAKE Indexing #
##################
FILE(GLOB_RECURSE LibFiles "include/*")
add_custom_target(headers SOURCES ${LibFiles})

cs_install()
cs_export()
