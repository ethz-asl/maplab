tensorflow_catkin
==============

Catkin package wrapper for Tensorflow 1.8 C++

- Uses the CMake build and does __not__ require Bazel
- Builds the [latest commit](https://github.com/tensorflow/tensorflow/tree/c1d223de41838e9d387a48137c76ea39d3b38f3f) of the master branch as of 24/06/2018
- Supports GPU using the `-DUSE_GPU` CMake flag
  - Assumes that the root directories of CUDA, CUDNN and NCCL are provided either as environment variables `CUDA_ROOT`, `CUDNN_ROOT` and `NCCL_ROOT`, or as flags `-DCUDA_ROOT`, `-DCUDNN_ROOT`, `-DNCCL_ROOT`
  - Supports CUDA >=8.0 and CUDNN >=6.0 (`-DCUDA_VERSION` and `-DCUDNN_VERSION` flags)
  - Supports linking against the run-time version of `libcuda` in `$CUDA_ROOT/lib64/stubs/`
- Can leverage several make jobs to speed up the build (`-DNUM_MAKE_JOBS` flag)
- Builds without SSE and AVX accelerations by default
  - Because of memory alignment issues in Eigen, it is sometimes necessary to build all the libraries of a project with the same flags
  - Set `-DCMAKE_CXX_FLAGS="-march=native"` to enable all the acceleration primitives
- Tested on Ubuntu 14 and 16 and on an NVIDIA Jetson TX2
- Requires CMake >=3.5
  
__Note__: some features are disabled, such as GRPC support, Python bindings, and JPEG- and PNG-related ops like image encoding and decoding (the latter interferes with OpenCV but can be enabled by deleting the relevant patch).

## Usage

#### package.xml
```xml
<?xml version="1.0" encoding="UTF-8"?>
<package format="2">
  <name>test_tensorflow</name>
  <version>0.0.0</version>
  <description>test_tensorflow</description>
  <maintainer email="some@example.com">Author</maintainer>
  <license>Apache 2.0</license>

  <buildtool_depend>catkin_simple</buildtool_depend>
  <buildtool_depend>catkin</buildtool_depend>
  <depend>tensorflow_catkin</depend>
</package>
```

#### CMakeLists.txt
```CMake
cmake_minimum_required(VERSION 2.8)
project(test_tensorflow)

find_package(catkin_simple REQUIRED)
catkin_simple()

cs_add_executable(example example.cpp)

cs_install()
cs_export()
```

#### example.cpp
```C++
#include <tensorflow/core/public/session.h>
#include <tensorflow/core/platform/env.h>
#include <iostream>
using namespace std;
using namespace tensorflow;

int main()
{
    Session* session;
    Status status = NewSession(SessionOptions(), &session);
    if (!status.ok()) {
        cout << status.ToString() << endl;
        return 1;
    }
    cout << "Session successfully created." << endl;
}
```

