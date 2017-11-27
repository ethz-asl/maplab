cmake_minimum_required(VERSION 2.8.3)
project(map_optimization)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

########
# LIB  #
########
cs_add_library(${PROJECT_NAME} 
  src/augment-loopclosure.cc
  src/optimization-problem.cc
  src/optimization-state-buffer.cc
  src/optimization-terms-addition.cc
  src/outlier-rejection-solver.cc
  src/solver.cc
  src/solver-options.cc
  src/vi-map-optimizer.cc
  src/vi-map-relaxation.cc
  src/vi-optimization-builder.cc
)

#############
## TESTING ##
#############
catkin_add_gtest(test_optimization test/test-optimization.cc)
target_link_libraries(test_optimization ${PROJECT_NAME})
maplab_import_test_maps(test_optimization)

##########
# EXPORT #
##########
cs_install()
cs_export()
