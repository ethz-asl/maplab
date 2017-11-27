/*
* Copyright (C) 2014 Simon Lynen, ASL, ETH Zurich, Switzerland
* You can contact the author at <slynen at ethz dot ch>
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
* http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*
* NOTICE:
* The abstraction of the entry point into class and base class has been done by
* Titus Cieslewski,
* ASL, ETH Zurich, Switzerland in 2015.
*/
#ifndef MAPLAB_COMMON_TESTING_ENTRYPOINT_H_
#define MAPLAB_COMMON_TESTING_ENTRYPOINT_H_

#include <gflags/gflags.h>
#include <glog/logging.h>
#include <gtest/gtest.h>

// Let the Eclipse parser see the macro.
#ifndef TEST
#define TEST(a, b) int Test_##a##_##b()
#endif

#ifndef TEST_F
#define TEST_F(a, b) int Test_##a##_##b()
#endif

#ifndef TEST_P
#define TEST_P(a, b) int Test_##a##_##b()
#endif

#ifndef TYPED_TEST
#define TYPED_TEST(a, b) int Test_##a##_##b()
#endif

#ifndef TYPED_TEST_P
#define TYPED_TEST_P(a, b) int Test_##a##_##b()
#endif

#ifndef TYPED_TEST_CASE
#define TYPED_TEST_CASE(a, b) int Test_##a##_##b()
#endif

#ifndef REGISTER_TYPED_TEST_CASE_P
#define REGISTER_TYPED_TEST_CASE_P(a, ...) int Test_##a()
#endif

#ifndef INSTANTIATE_TYPED_TEST_CASE_P
#define INSTANTIATE_TYPED_TEST_CASE_P(a, ...) int Test_##a()
#endif

namespace common {

class UnitTestEntryPointBase {
 public:
  virtual ~UnitTestEntryPointBase();
  // This function must be inline to avoid linker errors.
  inline int run(int argc, char** argv) {
    google::InitGoogleLogging(argv[0]);
    google::InstallFailureSignalHandler();
    testing::InitGoogleTest(&argc, argv);
    google::ParseCommandLineFlags(&argc, &argv, true);
    ::testing::FLAGS_gtest_death_test_style = "threadsafe";
    FLAGS_alsologtostderr = true;
    FLAGS_colorlogtostderr = true;
    FLAGS_v = 1;
    customInit();
    return RUN_ALL_TESTS();
  }

 private:
  virtual void customInit() = 0;
};

class UnitTestEntryPoint : public UnitTestEntryPointBase {
 public:
  virtual ~UnitTestEntryPoint();

 private:
  virtual void customInit();
};

}  // namespace common

#define MAPLAB_UNITTEST_ENTRYPOINT          \
  int main(int argc, char** argv) {         \
    common::UnitTestEntryPoint entry_point; \
    return entry_point.run(argc, argv);     \
  }

#endif  // MAPLAB_COMMON_TESTING_ENTRYPOINT_H_
