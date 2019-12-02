cmake_minimum_required(VERSION 2.8.3)
project(localization_fusion)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

cs_add_library(${PROJECT_NAME}
        src/filter-base.cc
        src/filter-utilities.cc
        src/localization-filter.cc
        src/ukf.cc)

##########
# GTESTS #
##########
catkin_add_gtest(test_ukf_localizer test/test-ukf-filter.cc)
target_link_libraries(test_ukf_localizer ${PROJECT_NAME})

## to add the data for localization fusion testing
SET(PROJECT_TEST_DATA "localization_fusion_test_data")
add_custom_target(${PROJECT_TEST_DATA})
add_custom_command(TARGET ${PROJECT_TEST_DATA}
COMMAND rm -rf "./${PROJECT_TEST_DATA}/*" && tar -xvzf ${MAPLAB_TEST_DATA_DIR}/${PROJECT_TEST_DATA}/${PROJECT_TEST_DATA}.tar.gz)

catkin_add_gtest(test_ukf_localizer_2d test/test-ukf-2d-performance.cc)
target_link_libraries(test_ukf_localizer_2d ${PROJECT_NAME})
add_dependencies(test_ukf_localizer_2d ${PROJECT_TEST_DATA})

cs_install()
cs_export()
