cmake_minimum_required(VERSION 3.5.0)
project(tensorflow_catkin)

find_package(catkin_simple REQUIRED)
catkin_simple()

set(USE_GPU OFF CACHE BOOL "Build Tensorflow with GPU support")
set(CUDA_ROOT $ENV{CUDA_ROOT} CACHE PATH "CUDA root directory")
set(CUDNN_ROOT $ENV{CUDNN_ROOT} CACHE PATH "CUDNN root directory")
set(NCCL_ROOT $ENV{NCCL_ROOT} CACHE PATH "NCCL root directory")
set(NUM_MAKE_JOBS "" CACHE STRING "Number of jobs concurrently run during the build (passed as make -j parameter)")
set(CUDA_VERSION $ENV{CUDA_VERSION} CACHE STRING "CUDA version")
set(CUDNN_VERSION $ENV{CUDNN_VERSION} CACHE STRING "CUDNN version")

if(${CMAKE_VERSION} VERSION_LESS "3.7.0")
       # ExternalProject_Add doesn't know the option SOURCE_SUBDIR in CMake before 3.7
       file(DOWNLOAD "https://raw.githubusercontent.com/Kitware/CMake/v3.8.0/Modules/ExternalProject.cmake" ${CMAKE_CURRENT_SOURCE_DIR}/ExternalProject.cmake)
       include(${CMAKE_CURRENT_SOURCE_DIR}/ExternalProject.cmake)
else()
  include(ExternalProject)
endif()

file(MAKE_DIRECTORY ${CATKIN_DEVEL_PREFIX}/include)

ExternalProject_Add(tensorflow_src
  GIT_REPOSITORY https://github.com/tensorflow/tensorflow.git
  GIT_TAG c1d223de41838e9d387a48137c76ea39d3b38f3f
  PATCH_COMMAND bash -c "for i in ${PROJECT_SOURCE_DIR}/patches/*.patch\$<SEMICOLON> do git apply -p1 $i\$<SEMICOLON> done"
  UPDATE_COMMAND ""
  BUILD_COMMAND make -j${NUM_MAKE_JOBS} install
  INSTALL_COMMAND echo "Install"
  SOURCE_DIR tensorflow_src
  SOURCE_SUBDIR tensorflow/contrib/cmake
  BINARY_DIR tensorflow_build
  CMAKE_ARGS
    -DCMAKE_CXX_FLAGS=${CMAKE_CXX_FLAGS}
    -DCMAKE_BUILD_TYPE=Release
    -DCMAKE_INSTALL_PREFIX=${CATKIN_DEVEL_PREFIX}
    # General TF options
    -Dtensorflow_BUILD_SHARED_LIB=ON
    -Dtensorflow_BUILD_ALL_KERNELS=ON
    -Dtensorflow_BUILD_CONTRIB_KERNELS=ON
    -Dtensorflow_ENABLE_GRPC_SUPPORT=OFF
    -Dtensorflow_ENABLE_SSL_SUPPORT=OFF
    -Dtensorflow_BUILD_CC_TESTS=OFF
    -Dtensorflow_BUILD_CC_EXAMPLE=OFF
    -Dtensorflow_BUILD_PYTHON_BINDINGS=OFF
    -Dtensorflow_BUILD_PYTHON_TESTS=OFF
    -Dtensorflow_OPTIMIZE_FOR_NATIVE_ARCH=OFF
    # GPU-specific options
    -Dtensorflow_ENABLE_GPU=${USE_GPU}
    -Dtensorflow_PATH_STATIC_LIB=${CUDA_ROOT}/lib64
    -Dtensorflow_CUDA_LIBRARY_PATH=${CUDA_ROOT}/lib64
    -DCUDA_CUDA_LIBRARY=${CUDA_ROOT}/lib64/stubs/libcuda.so
    -Dtensorflow_PATH_NCCL_STATIC_LIB=${NCCL_ROOT}/lib
    -Dtensorflow_CUDNN_INCLUDE=${CUDNN_ROOT}/include
    -Dtensorflow_PATH_CUDNN_STATIC_LIB=${CUDNN_ROOT}/lib64
    -Dtensorflow_CUDA_VERSION=${CUDA_VERSION}
    -Dtensorflow_CUDNN_VERSION=${CUDNN_VERSION}
)

cs_add_library(${PROJECT_NAME} src/dependency_tracker.cc)
add_dependencies(${PROJECT_NAME} tensorflow_src)
target_link_libraries(${PROJECT_NAME}
  ${CATKIN_DEVEL_PREFIX}/lib/libtensorflow${CMAKE_SHARED_LIBRARY_SUFFIX})

cs_install()

cs_export(
  INCLUDE_DIRS ${CATKIN_DEVEL_PREFIX}/include
CFG_EXTRAS tensorflow-extras.cmake)
