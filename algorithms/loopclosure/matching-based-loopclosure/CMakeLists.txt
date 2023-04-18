cmake_minimum_required(VERSION 2.8)
project(matching_based_loopclosure)

find_package(catkin_simple REQUIRED)
catkin_simple()

set(LIBRARY_NAME ${PROJECT_NAME})
cs_add_library(${LIBRARY_NAME} src/detector-settings.cc
                               src/matching-based-engine.cc
                               src/train-vocabulary.cc)

# Install loopclosure files and export their location to the Catkin environment.
set(MAPLAB_LOOPCLOSURE_DESTINATION ${CATKIN_GLOBAL_SHARE_DESTINATION}/loopclosure)
set(FILES_TO_COPY share/inverted_multi_index_quantizer_freak.dat share/projection_matrix_freak.dat
                  share/inverted_multi_index_quantizer_brisk.dat share/projection_matrix_brisk.dat)
file(COPY ${FILES_TO_COPY}
     DESTINATION ${CATKIN_DEVEL_PREFIX}/${MAPLAB_LOOPCLOSURE_DESTINATION})
install(FILES ${FILES_TO_COPY}
        DESTINATION ${MAPLAB_LOOPCLOSURE_DESTINATION})
catkin_add_env_hooks(40.loopclosure
                     SHELLS sh
                     DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/env-hooks)

catkin_add_gtest(test_scoring test/test_scoring.cc)
target_link_libraries(test_scoring ${LIBRARY_NAME})

cs_install()
cs_export()
