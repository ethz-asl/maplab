#include <maplab-common/parallel-process.h>
#include <maplab-common/test/testing-entrypoint.h>

namespace common {
struct Squarer {
  Squarer(const std::vector<double>& input, std::vector<double>* output)
      : input_(input), output_(CHECK_NOTNULL(output)) {}
  const std::vector<double>& input_;
  std::vector<double>* output_;
  void operator()(const std::vector<size_t>& range) const {
    for (size_t i : range) {
      (*output_)[i] = input_[i] * input_[i];
    }
  }
};

TEST(MaplabCommon, ParallelProcessEvenNumber8Threads) {
  std::vector<double> data, results;
  constexpr int kNumValues = 12;
  for (int i = 0; i < kNumValues; ++i) {
    data.push_back(i);
  }
  results.resize(kNumValues, -1);

  Squarer squarer(data, &results);
  ParallelProcess(data.size(), squarer, true, 8);
  for (int i = 0; i < kNumValues; ++i) {
    EXPECT_EQ(results[i], data[i] * data[i]);
  }
}

TEST(MaplabCommon, ParallelProcessOddNumberOddNumThreads) {
  std::vector<double> data, results;
  constexpr int kNumValues = 13;
  for (int i = 0; i < kNumValues; ++i) {
    data.push_back(i);
  }
  results.resize(kNumValues, -1);

  Squarer squarer(data, &results);
  ParallelProcess(data.size(), squarer, true, 7);
  for (int i = 0; i < kNumValues; ++i) {
    EXPECT_EQ(results[i], data[i] * data[i]);
  }
}

TEST(MaplabCommon, ParallelProcessOddNumberEvenNumThreads) {
  std::vector<double> data, results;
  constexpr int kNumValues = 13;
  for (int i = 0; i < kNumValues; ++i) {
    data.push_back(i);
  }
  results.resize(kNumValues, -1);

  Squarer squarer(data, &results);
  ParallelProcess(data.size(), squarer, true, 16);
  for (int i = 0; i < kNumValues; ++i) {
    EXPECT_EQ(results[i], data[i] * data[i]);
  }
}

TEST(MaplabCommon, ParallelProcessOddNumberOneThread) {
  std::vector<double> data, results;
  constexpr int kNumValues = 13;
  for (int i = 0; i < kNumValues; ++i) {
    data.push_back(i);
  }
  results.resize(kNumValues, -1);

  Squarer squarer(data, &results);
  ParallelProcess(data.size(), squarer, true, 1);
  for (int i = 0; i < kNumValues; ++i) {
    EXPECT_EQ(results[i], data[i] * data[i]);
  }
}
}  // namespace common

MAPLAB_UNITTEST_ENTRYPOINT
