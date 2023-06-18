#include "aslam/common/statistics/statistics.h"

#include <cmath>
#include <fstream>  // NOLINT
#include <ostream>  // NOLINT
#include <sstream>

namespace statistics {

Statistics& Statistics::Instance() {
  static Statistics instance;
  return instance;
}

Statistics::Statistics() : max_tag_length_(0) {}

Statistics::~Statistics() {}

// Static functions to query the stats collectors:
size_t Statistics::GetHandle(std::string const& tag) {
  std::lock_guard<std::mutex> lock(Instance().mutex_);
  // Search for an existing tag.
  map_t::iterator i = Instance().tag_map_.find(tag);
  if (i == Instance().tag_map_.end()) {
    // If it is not there, create a tag.
    size_t handle = Instance().stats_collectors_.size();
    Instance().tag_map_[tag] = handle;
    Instance().stats_collectors_.push_back(StatisticsMapValue());
    // Track the maximum tag length to help printing a table of values later.
    Instance().max_tag_length_ =
        std::max(Instance().max_tag_length_, tag.size());
    return handle;
  } else {
    return i->second;
  }
}

// Return true if a handle has been initialized for a specific tag.
// In contrast to GetHandle(), this allows testing for existence without
// modifying the tag/handle map.
bool Statistics::HasHandle(std::string const& tag) {
  std::lock_guard<std::mutex> lock(Instance().mutex_);
  bool tag_found = Instance().tag_map_.count(tag);
  return tag_found;
}

std::string Statistics::GetTag(size_t handle) {
  std::lock_guard<std::mutex> lock(Instance().mutex_);
  std::string tag;
  // Perform a linear search for the tag.
  for (typename map_t::value_type current_tag : Instance().tag_map_) {
    if (current_tag.second == handle) {
      return current_tag.first;
    }
  }
  return tag;
}

StatsCollectorImpl::StatsCollectorImpl(size_t handle) : handle_(handle) {}

StatsCollectorImpl::StatsCollectorImpl(std::string const& tag)
    : handle_(Statistics::GetHandle(tag)) {}

size_t StatsCollectorImpl::GetHandle() const {
  return handle_;
}
void StatsCollectorImpl::AddSample(double sample) const {
  Statistics::Instance().AddSample(handle_, sample);
}
void StatsCollectorImpl::IncrementOne() const {
  Statistics::Instance().AddSample(handle_, 1.0);
}
void Statistics::AddSample(size_t handle, double seconds) {
  std::lock_guard<std::mutex> lock(Instance().mutex_);
  stats_collectors_[handle].AddValue(seconds);
}
double Statistics::GetLastValue(size_t handle) {
  std::lock_guard<std::mutex> lock(Instance().mutex_);
  return Instance().stats_collectors_[handle].GetLastValue();
}
double Statistics::GetLastValue(std::string const& tag) {
  return GetLastValue(GetHandle(tag));
}
double Statistics::GetTotal(size_t handle) {
  std::lock_guard<std::mutex> lock(Instance().mutex_);
  return Instance().stats_collectors_[handle].Sum();
}
double Statistics::GetTotal(std::string const& tag) {
  return GetTotal(GetHandle(tag));
}
double Statistics::GetMean(size_t handle) {
  std::lock_guard<std::mutex> lock(Instance().mutex_);
  return Instance().stats_collectors_[handle].Mean();
}
double Statistics::GetMean(std::string const& tag) {
  return GetMean(GetHandle(tag));
}
size_t Statistics::GetNumSamples(size_t handle) {
  std::lock_guard<std::mutex> lock(Instance().mutex_);
  return Instance().stats_collectors_[handle].TotalSamples();
}
size_t Statistics::GetNumSamples(std::string const& tag) {
  return GetNumSamples(GetHandle(tag));
}
double Statistics::GetVariance(size_t handle) {
  std::lock_guard<std::mutex> lock(Instance().mutex_);
  return Instance().stats_collectors_[handle].LazyVariance();
}
double Statistics::GetVariance(std::string const& tag) {
  return GetVariance(GetHandle(tag));
}
double Statistics::GetMin(size_t handle) {
  std::lock_guard<std::mutex> lock(Instance().mutex_);
  return Instance().stats_collectors_[handle].Min();
}
double Statistics::GetMin(std::string const& tag) {
  return GetMin(GetHandle(tag));
}
double Statistics::GetMax(size_t handle) {
  std::lock_guard<std::mutex> lock(Instance().mutex_);
  return Instance().stats_collectors_[handle].Max();
}
double Statistics::GetMax(std::string const& tag) {
  return GetMax(GetHandle(tag));
}
double Statistics::GetHz(size_t handle) {
  std::lock_guard<std::mutex> lock(Instance().mutex_);
  return Instance().stats_collectors_[handle].MeanCallsPerSec();
}
double Statistics::GetHz(std::string const& tag) {
  return GetHz(GetHandle(tag));
}

// Delta time statistics.
double Statistics::GetMeanDeltaTime(std::string const& tag) {
  return GetMeanDeltaTime(GetHandle(tag));
}
double Statistics::GetMeanDeltaTime(size_t handle) {
  std::lock_guard<std::mutex> lock(Instance().mutex_);
  return Instance().stats_collectors_[handle].MeanDeltaTime();
}
double Statistics::GetMaxDeltaTime(std::string const& tag) {
  return GetMaxDeltaTime(GetHandle(tag));
}
double Statistics::GetMaxDeltaTime(size_t handle) {
  std::lock_guard<std::mutex> lock(Instance().mutex_);
  return Instance().stats_collectors_[handle].MaxDeltaTime();
}
double Statistics::GetMinDeltaTime(std::string const& tag) {
  return GetMinDeltaTime(GetHandle(tag));
}
double Statistics::GetMinDeltaTime(size_t handle) {
  std::lock_guard<std::mutex> lock(Instance().mutex_);
  return Instance().stats_collectors_[handle].MinDeltaTime();
}
double Statistics::GetLastDeltaTime(std::string const& tag) {
  return GetLastDeltaTime(GetHandle(tag));
}
double Statistics::GetLastDeltaTime(size_t handle) {
  std::lock_guard<std::mutex> lock(Instance().mutex_);
  return Instance().stats_collectors_[handle].GetLastDeltaTime();
}
double Statistics::GetVarianceDeltaTime(std::string const& tag) {
  return GetVarianceDeltaTime(GetHandle(tag));
}
double Statistics::GetVarianceDeltaTime(size_t handle) {
  std::lock_guard<std::mutex> lock(Instance().mutex_);
  return Instance().stats_collectors_[handle].LazyVarianceDeltaTime();
}

std::string Statistics::SecondsToTimeString(double seconds) {
  double secs = fmod(seconds, 60);
  int minutes = (seconds / 60);
  int hours = (seconds / 3600);
  minutes = minutes - (hours * 60);

  char buffer[256];
  snprintf(
      buffer, sizeof(buffer),
#ifdef SM_TIMING_SHOW_HOURS
      "%02d:"
#endif
#ifdef SM_TIMING_SHOW_MINUTES
      "%02d:"
#endif
      "%09.6f",
#ifdef SM_TIMING_SHOW_HOURS
      hours,
#endif
#ifdef SM_TIMING_SHOW_MINUTES
      minutes,
#endif
      secs);
  return buffer;
}

void Statistics::Print(std::ostream& out) {  // NOLINT
  const map_t& tag_map = Instance().tag_map_;

  if (tag_map.empty()) {
    return;
  }

  out << "Statistics\n";

  out.width((std::streamsize)Instance().max_tag_length_);
  out.setf(std::ios::left, std::ios::adjustfield);
  out << "-----------";
  out.width(7);
  out.setf(std::ios::right, std::ios::adjustfield);
  out << "#\t";
  out << "Hz\t";
  out << "(avg     +- std    )\t";
  out << "[min,max]\n";

  for (const typename map_t::value_type& t : tag_map) {
    size_t i = t.second;
    out.width((std::streamsize)Instance().max_tag_length_);
    out.setf(std::ios::left, std::ios::adjustfield);
    out << t.first << "\t";
    out.width(7);

    out.setf(std::ios::right, std::ios::adjustfield);
    out << GetNumSamples(i) << "\t";
    if (GetNumSamples(i) > 0) {
      out << GetHz(i) << "\t";
      double mean = GetMean(i);
      double stddev = sqrt(GetVariance(i));
      out << "(" << mean << " +- ";
      out << stddev << ")\t";

      double min_value = GetMin(i);
      double max_value = GetMax(i);

      out << "[" << min_value << "," << max_value << "]";
    }
    out << std::endl;
  }
}

void Statistics::WriteToYamlFile(const std::string& path) {
  const map_t& tag_map = Instance().tag_map_;

  if (tag_map.empty()) {
    return;
  }

  std::ofstream output_file(path);

  if (!output_file) {
    LOG(ERROR) << "Could not write statistics: Unable to open file: " << path;
    return;
  }

  VLOG(1) << "Writing statistics to file: " << path;
  for (const map_t::value_type& tag : tag_map) {
    const size_t index = tag.second;

    if (GetNumSamples(index) > 0) {
      std::string label = tag.first;

      // We do not want colons or hashes in a label, as they might interfere
      // with reading the yaml later.
      std::replace(label.begin(), label.end(), ':', '_');
      std::replace(label.begin(), label.end(), '#', '_');

      output_file << label << ":" << "\n";
      output_file << "  samples: " << GetNumSamples(index) << "\n";
      output_file << "  mean: " << GetMean(index) << "\n";
      output_file << "  stddev: " << sqrt(GetVariance(index)) << "\n";
      output_file << "  min: " << GetMin(index) << "\n";
      output_file << "  max: " << GetMax(index) << "\n";
    }
    output_file << "\n";
  }
}

std::string Statistics::Print() {
  std::stringstream ss;
  Print(ss);
  return ss.str();
}

void Statistics::Reset() {
  std::lock_guard<std::mutex> lock(Instance().mutex_);
  Instance().tag_map_.clear();
}

}  // namespace statistics
