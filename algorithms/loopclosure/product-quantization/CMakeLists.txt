cmake_minimum_required(VERSION 2.8)
project(product_quantization)

find_package(catkin_simple REQUIRED)
catkin_simple()

set(LIBRARY_NAME ${PROJECT_NAME})
cs_add_library(${LIBRARY_NAME} src/learn-product-quantization.cc)

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(test_product-quantization
                   test/test_product-quantization.cc
                   WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/)
  target_link_libraries(test_product-quantization ${catkin_LIBRARIES})
  
  catkin_add_gtest(test_learn-product-quantization
                   test/test_learn-product-quantization.cc
                   WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/)
  target_link_libraries(test_learn-product-quantization ${LIBRARY_NAME}) 
endif()

cs_install()

cs_export()
