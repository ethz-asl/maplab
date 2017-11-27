#include "maplab-common/test/testing-entrypoint.h"
#include "maplab-common/threaded-task-queue-processor.h"

class TestClass {
 public:
  void testFunction(int number) {
    results.emplace_back(number);
  }
  std::vector<int> results;
};

class ThreadedTaskQueueProcessorFixture : public ::testing::Test {
 protected:
  virtual void SetUp() {}
  virtual void TearDown() {}

 protected:
  TestClass test_class;
};

const int kNumTestTask = 1000;

TEST_F(ThreadedTaskQueueProcessorFixture, ProcessAllSequentiallyPolicy) {
  const bool kStartProcessing = false;
  common::ThreadedTaskQueueProcessor<common::ProcessAllSequentiallyQueue>
      processor(kStartProcessing);
  for (int i = 0; i < kNumTestTask; ++i) {
    processor.addTask(&TestClass::testFunction, &(this->test_class), i);
  }
  processor.startProcessing();
  sleep(1);

  ASSERT_EQ(this->test_class.results.size(), kNumTestTask);
  for (int i = 0; i < kNumTestTask; ++i) {
    EXPECT_EQ(this->test_class.results[i], i);
  }
}

TEST_F(ThreadedTaskQueueProcessorFixture, ProcessNewestDropRestPolicy) {
  const bool kStartProcessing = false;
  common::ThreadedTaskQueueProcessor<common::ProcessNewestDropRestQueue>
      processor(kStartProcessing);
  for (int i = 0; i < kNumTestTask; ++i) {
    processor.addTask(&TestClass::testFunction, &(this->test_class), i);
  }
  processor.startProcessing();
  sleep(1);

  ASSERT_EQ(this->test_class.results.size(), 1u);
  EXPECT_EQ(this->test_class.results[0], kNumTestTask - 1);
}

MAPLAB_UNITTEST_ENTRYPOINT
