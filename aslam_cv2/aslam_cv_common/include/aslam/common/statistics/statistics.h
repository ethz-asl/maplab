#ifndef ASLAM_STATISTICS_H_
#define ASLAM_STATISTICS_H_

#include <chrono>
#include <map>
#include <mutex>
#include <string>
#include <vector>

#include "aslam/common/statistics/accumulator.h"

///
// Example usage:
//
// #define ENABLE_STATISTICS 1 // Turn on/off the statistics calculation
// #include <aslam/common/statistics/statistics.h>
//
// double my_distance = measureDistance();
// statistics::DebugStatsCollector distance_stat("Distance measurement");
// distance_stat.AddSample(my_distance);
//
// std::cout << statistics::Statistics::Print();
///

namespace statistics {

const double kNumSecondsPerNanosecond = 1.e-9;

struct StatisticsMapValue {
  static const int kWindowSize = 100;

  inline StatisticsMapValue()
      : time_last_called_(std::chrono::system_clock::from_time_t(0)),
        epoch_clock_(std::chrono::system_clock::from_time_t(0)) {}

  inline void AddValue(double sample) {
    std::chrono::time_point<std::chrono::system_clock> now =
        std::chrono::system_clock::now();

    // Only calculate delta time if it has been called before.
    if (time_last_called_ != epoch_clock_) {
      double dt = static_cast<double>(
                      std::chrono::duration_cast<std::chrono::nanoseconds>(
                          now - time_last_called_)
                          .count()) *
                  kNumSecondsPerNanosecond;
      time_deltas_.Add(dt);
    }

    time_last_called_ = now;
    values_.Add(sample);
  }
  inline double GetLastDeltaTime() const {
    if (time_deltas_.total_samples()) {
      return time_deltas_.GetMostRecent();
    } else {
      return 0;
    }
  }
  inline double GetLastValue() const {
    if (values_.total_samples()) {
      return values_.GetMostRecent();
    } else {
      return 0;
    }
  }
  inline double Sum() const {
    return values_.sum();
  }
  int TotalSamples() const {
    return values_.total_samples();
  }
  double Mean() const {
    return values_.Mean();
  }
  double RollingMean() const {
    return values_.RollingMean();
  }
  double Max() const {
    return values_.max();
  }
  double Min() const {
    return values_.min();
  }
  double LazyVariance() const {
    return values_.LazyVariance();
  }
  double MeanCallsPerSec() const {
    double mean_dt = time_deltas_.Mean();
    if (mean_dt != 0) {
      return 1.0 / mean_dt;
    } else {
      return -1.0;
    }
  }

  double MeanDeltaTime() const {
    return time_deltas_.Mean();
  }
  double RollingMeanDeltaTime() const {
    return time_deltas_.RollingMean();
  }
  double MaxDeltaTime() const {
    return time_deltas_.max();
  }
  double MinDeltaTime() const {
    return time_deltas_.min();
  }
  double LazyVarianceDeltaTime() const {
    return time_deltas_.LazyVariance();
  }

 private:
  // Create an accumulator with specified window size.
  Accumulator<double, double, kWindowSize> values_;
  Accumulator<double, double, kWindowSize> time_deltas_;
  std::chrono::time_point<std::chrono::system_clock> time_last_called_;
  std::chrono::time_point<std::chrono::system_clock> epoch_clock_;
};

// A class that has the statistics interface but does nothing. Swapping this in
// in place of the Statistics class (say with a typedef) eliminates the function
// calls.
class DummyStatsCollector {
 public:
  explicit DummyStatsCollector(size_t /*handle*/) {}
  explicit DummyStatsCollector(std::string const& /*tag*/) {}
  void AddSample(double /*sample*/) const {}
  void IncrementOne() const {}
  size_t GetHandle() const {
    return 0u;
  }
};

class StatsCollectorImpl {
 public:
  explicit StatsCollectorImpl(size_t handle);
  explicit StatsCollectorImpl(std::string const& tag);
  ~StatsCollectorImpl() = default;

  void AddSample(double sample) const;
  void IncrementOne() const;
  size_t GetHandle() const;

 private:
  size_t handle_;
};

class Statistics {
 public:
  typedef std::map<std::string, size_t> map_t;
  friend class StatsCollectorImpl;
  // Definition of static functions to query the stats.
  static size_t GetHandle(std::string const& tag);
  static bool HasHandle(std::string const& tag);
  static std::string GetTag(size_t handle);
  static double GetLastValue(size_t handle);
  static double GetLastValue(std::string const& tag);
  static double GetTotal(size_t handle);
  static double GetTotal(std::string const& tag);
  static double GetMean(size_t handle);
  static double GetMean(std::string const& tag);
  static size_t GetNumSamples(size_t handle);
  static size_t GetNumSamples(std::string const& tag);
  static double GetVariance(size_t handle);
  static double GetVariance(std::string const& tag);
  static double GetMin(size_t handle);
  static double GetMin(std::string const& tag);
  static double GetMax(size_t handle);
  static double GetMax(std::string const& tag);
  static double GetHz(size_t handle);
  static double GetHz(std::string const& tag);

  static double GetMeanDeltaTime(std::string const& tag);
  static double GetMeanDeltaTime(size_t handle);
  static double GetMaxDeltaTime(std::string const& tag);
  static double GetMaxDeltaTime(size_t handle);
  static double GetMinDeltaTime(std::string const& tag);
  static double GetMinDeltaTime(size_t handle);
  static double GetLastDeltaTime(std::string const& tag);
  static double GetLastDeltaTime(size_t handle);
  static double GetVarianceDeltaTime(std::string const& tag);
  static double GetVarianceDeltaTime(size_t handle);

  static void WriteToYamlFile(const std::string& path);
  static void Print(std::ostream& out);  // NOLINT
  static std::string Print();
  static std::string SecondsToTimeString(double seconds);
  static void Reset();
  static const map_t& GetStatsCollectors() {
    return Instance().tag_map_;
  }

 private:
  void AddSample(size_t handle, double sample);

  static Statistics& Instance();

  Statistics();
  ~Statistics();

  typedef std::vector<statistics::StatisticsMapValue> list_t;

  list_t stats_collectors_;
  map_t tag_map_;
  size_t max_tag_length_;
  std::mutex mutex_;
};

#if ENABLE_STATISTICS
typedef StatsCollectorImpl StatsCollector;
#else
typedef DummyStatsCollector StatsCollector;
#endif

}  // namespace statistics

#endif  // ASLAM_STATISTICS_H_
