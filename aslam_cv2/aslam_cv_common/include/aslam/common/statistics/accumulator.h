#ifndef ASLAM_STATISTICS_ACCUMULATOR_H_
#define ASLAM_STATISTICS_ACCUMULATOR_H_

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

#include <glog/logging.h>

namespace statistics {
static constexpr int kInfiniteWindowSize = std::numeric_limits<int>::max();
// If the window size is set to -1, the vector will grow infinitely, otherwise,
// the vector has a fixed size.
template <typename SampleType, typename SumType, int WindowSize>
class Accumulator {
 public:
  Accumulator()
      : sample_index_(0),
        total_samples_(0),
        sum_(0),
        window_sum_(0),
        min_(std::numeric_limits<SampleType>::max()),
        max_(std::numeric_limits<SampleType>::lowest()),
        most_recent_(0) {
    CHECK_GT(WindowSize, 0);
    if (WindowSize < kInfiniteWindowSize) {
      samples_.reserve(WindowSize);
    }
  }

  void Add(SampleType sample) {
    most_recent_ = sample;
    if (sample_index_ < WindowSize) {
      samples_.push_back(sample);
      window_sum_ += sample;
      ++sample_index_;
    } else {
      SampleType& oldest = samples_.at(sample_index_++ % WindowSize);
      window_sum_ += sample - oldest;
      oldest = sample;
    }
    sum_ += sample;
    ++total_samples_;
    if (sample > max_) {
      max_ = sample;
    }
    if (sample < min_) {
      min_ = sample;
    }
  }

  int total_samples() const {
    return total_samples_;
  }

  SumType sum() const {
    return sum_;
  }

  SumType Mean() const {
    return (total_samples_ < 1) ? 0.0 : sum_ / total_samples_;
  }

  // Rolling mean is only used for fixed sized data for now. We don't need this
  // function for our infinite accumulator at this point.
  SumType RollingMean() const {
    if (WindowSize < kInfiniteWindowSize) {
      return window_sum_ / std::min(sample_index_, WindowSize);
    } else {
      return Mean();
    }
  }

  SampleType GetMostRecent() const {
    return most_recent_;
  }

  SumType max() const {
    return max_;
  }

  SumType min() const {
    return min_;
  }

  SumType LazyVariance() const {
    if (samples_.size() < 2) {
      return 0.0;
    }

    SumType var = static_cast<SumType>(0.0);
    SumType mean = RollingMean();

    for (unsigned int i = 0; i < samples_.size(); ++i) {
      var += (samples_[i] - mean) * (samples_[i] - mean);
    }

    var /= samples_.size() - 1;
    return var;
  }

  SumType StandardDeviation() const {
    return std::sqrt(LazyVariance());
  }

  const std::vector<SampleType>& GetSamples() const {
    return samples_;
  }

 private:
  std::vector<SampleType> samples_;
  int sample_index_;
  int total_samples_;
  SumType sum_;
  SumType window_sum_;
  SampleType min_;
  SampleType max_;
  SampleType most_recent_;
};

typedef Accumulator<double, double, kInfiniteWindowSize> Accumulatord;

}  // namespace statistics

#endif  // ASLAM_STATISTICS_ACCUMULATOR_H_
