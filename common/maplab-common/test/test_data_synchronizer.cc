#include <chrono>
#include <random>
#include <thread>
#include <unordered_map>
#include <utility>
#include <vector>

#include "maplab-common/data-synchronizer.h"
#include "maplab-common/test/testing-entrypoint.h"

constexpr size_t kRandomSeed = 42;

struct TestDataA {
  int64_t timestamp;
};
struct TestDataB {
  int64_t timestamp_ns;
};

struct TimestampExtractorA {
  int64_t operator()(const TestDataA& data) {
    return data.timestamp;
  }
};

typedef common::DataSynchronizer<TestDataA, TestDataB, TimestampExtractorA>
  DataSynchronizer;

bool sampleBoolUniform(double outlier_probability) {
  static std::mt19937 gen(kRandomSeed);
  static std::uniform_real_distribution<> dis(0.0, 1.0);
  return dis(gen) <= outlier_probability;
}

void generateTestData(
    size_t n_samples, double drop_probability,
    std::vector<TestDataA>* data_a, std::vector<TestDataB>* data_b,
    std::vector<std::pair<size_t, size_t>>* ground_truth_matches_indices_ab) {
  CHECK_NOTNULL(data_a);
  CHECK_NOTNULL(data_b);
  CHECK_NOTNULL(ground_truth_matches_indices_ab);

  for (size_t i = 0; i < n_samples; ++i) {
    bool both_data_available = true;
    if (!sampleBoolUniform(drop_probability)) {
      data_a->emplace_back(TestDataA{.timestamp = static_cast<int64_t>(i)});
    } else {
      both_data_available = false;
    }

    if (!sampleBoolUniform(drop_probability)) {
      data_b->emplace_back(TestDataB{.timestamp_ns = static_cast<int64_t>(i)});
    } else {
      both_data_available = false;
    }

    if (both_data_available) {
      ground_truth_matches_indices_ab->emplace_back(
          data_a->size() - 1u, data_b->size() - 1u);
    }
  }

  // Make sure there were some drops, so that we actually test the logic as
  // intended.
  CHECK((data_a->size() > ground_truth_matches_indices_ab->size()) ||
        (data_b->size() > ground_truth_matches_indices_ab->size()));
}

typedef std::function<void(const std::vector<TestDataA>&,
                           const std::vector<TestDataB>&,
                           DataSynchronizer*)> DataFeederFunction;
void runTest(const DataFeederFunction& data_feeder_function) {
  // Generate some test data.
  constexpr size_t kNumSamples = 1000;
  constexpr double kDropProbability = 0.15;
  std::vector<TestDataA> data_a;
  std::vector<TestDataB> data_b;
  std::vector<std::pair<size_t, size_t>> expected_matching_indices_ab;
  generateTestData(kNumSamples, kDropProbability, &data_a, &data_b,
                   &expected_matching_indices_ab);

  // Now try to synchronize the data.
  DataSynchronizer data_synchronizer;

  std::vector<std::pair<TestDataA, TestDataB>> result_queue;
  data_synchronizer.registerCallback([&result_queue](
      const TestDataA& data_a, const TestDataB& data_b) {
    result_queue.emplace_back(data_a, data_b);
  });
  data_feeder_function(data_a, data_b, &data_synchronizer);

  // Check the results.
  ASSERT_EQ(result_queue.size(), expected_matching_indices_ab.size());

  for (size_t i = 0; i < result_queue.size(); ++i) {
    const size_t index_a = expected_matching_indices_ab[i].first;
    const size_t index_b = expected_matching_indices_ab[i].second;
    CHECK_LT(index_a, data_a.size());
    CHECK_LT(index_b, data_b.size());

    EXPECT_EQ(result_queue[i].first.timestamp, data_a[index_a].timestamp);
    EXPECT_EQ(result_queue[i].second.timestamp_ns, data_b[index_b].timestamp_ns);
  }

  // After a successful match the queues are expected to be empty.
  data_synchronizer.processTypeA(TestDataA{55000000});
  data_synchronizer.processTypeB(TestDataB{55000000});

  EXPECT_EQ(data_synchronizer.getContainerSizes().first, 0);
  EXPECT_EQ(data_synchronizer.getContainerSizes().second, 0);
}

TEST(DataSynchronizer, DataSynchronization) {
  DataFeederFunction synchronous_data_feeding = [](
      const std::vector<TestDataA>& data_a,
      const std::vector<TestDataB>& data_b,
      DataSynchronizer* data_synchronizer) {
    CHECK_NOTNULL(data_synchronizer);
    // Alternate between A and B data insertion randomly.
    size_t idx_a = 0;
    size_t idx_b = 0;
    while (idx_a < data_a.size() || idx_b < data_b.size()) {
      if (sampleBoolUniform(0.25) && idx_a < data_a.size()) {
        CHECK_LT(idx_a, data_a.size());
        data_synchronizer->processTypeA(data_a[idx_a++]);
      } else if (idx_b < data_b.size()) {
        CHECK_LT(idx_b, data_b.size());
        data_synchronizer->processTypeB(data_b[idx_b++]);
      }
    }
    CHECK_EQ(idx_a, data_a.size());
    CHECK_EQ(idx_b, data_b.size());
  };
  runTest(synchronous_data_feeding);
}

TEST(DataSynchronizer, DataSynchronization_Async) {
  DataFeederFunction async_data_feeding = [](
      const std::vector<TestDataA>& data_a,
      const std::vector<TestDataB>& data_b,
      DataSynchronizer* data_synchronizer) {
    CHECK_NOTNULL(data_synchronizer);

    auto waitRandomTime = []() {
      static thread_local std::mt19937 generator(kRandomSeed);
      std::uniform_int_distribution<uint64_t> distribution(0,100);
      uint64_t sleep_us = distribution(generator);
      std::this_thread::sleep_for(std::chrono::microseconds(sleep_us));
    };

    std::thread thread_a([&]() {
      for (const TestDataA& elem : data_a) {
        data_synchronizer->processTypeA(elem);
        waitRandomTime();
      }
    });

    std::thread thread_b([&]() {
      for (const TestDataB& elem : data_b) {
        data_synchronizer->processTypeB(elem);
        waitRandomTime();
      }
    });
    thread_a.join();
    thread_b.join();
  };
  runTest(async_data_feeding);
}

TEST(DataSynchronizer, DeathOnNonIncreasingTimestamp) {
  // Generate some test data.
  constexpr size_t kNumSamples = 1000;
  constexpr double kDropProbability = 0.1;
  std::vector<TestDataA> data_a;
  std::vector<TestDataB> data_b;
  std::vector<std::pair<size_t, size_t>> expected_matching_indices_ab;
  generateTestData(kNumSamples, kDropProbability, &data_a, &data_b,
                   &expected_matching_indices_ab);

  // Add some data in order.
  common::DataSynchronizer<TestDataA, TestDataB, TimestampExtractorA>
    data_synchronizer;

  data_synchronizer.processTypeA(data_a[100]);
  data_synchronizer.processTypeA(data_a[101]);

  data_synchronizer.processTypeB(data_b[50]);
  data_synchronizer.processTypeB(data_b[53]);

  // Then out-of-order where we expect death.
  EXPECT_DEATH(data_synchronizer.processTypeA(data_a[100]),"");
  EXPECT_DEATH(data_synchronizer.processTypeA(data_a[99]),"");
  EXPECT_DEATH(data_synchronizer.processTypeB(data_b[50]),"");
  EXPECT_DEATH(data_synchronizer.processTypeB(data_b[49]),"");
}

MAPLAB_UNITTEST_ENTRYPOINT
