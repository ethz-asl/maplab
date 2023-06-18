#include "aslam/common/timer.h"

#include <algorithm>
#include <fstream>  // NOLINT
#include <math.h>
#include <ostream>  //NOLINT
#include <sstream>
#include <stdio.h>
#include <string>

namespace timing {

const double kNumSecondsPerNanosecond = 1.e-9;

Timing& Timing::Instance() {
  static Timing t;
  return t;
}

Timing::Timing() : max_tag_length_(0u) {}

Timing::~Timing() {}

// Static functions to query the timers:
size_t Timing::GetHandle(const std::string& tag) {
  std::lock_guard<std::mutex> lock(Instance().mutex_);
  // Search for an existing tag.
  map_t::iterator tag_iterator = Instance().tag_map_.find(tag);
  if (tag_iterator == Instance().tag_map_.end()) {
    // If it is not there, create a tag.
    size_t handle = Instance().timers_.size();
    Instance().tag_map_[tag] = handle;
    Instance().timers_.push_back(statistics::StatisticsMapValue());
    // Track the maximum tag length to help printing a table of timing values
    // later.
    Instance().max_tag_length_ =
        std::max(Instance().max_tag_length_, tag.size());
    return handle;
  } else {
    return tag_iterator->second;
  }
}

std::string Timing::GetTag(size_t handle) {
  std::lock_guard<std::mutex> lock(Instance().mutex_);
  std::string tag;

  // Perform a linear search for the tag.
  for (const typename map_t::value_type& current_tag : Instance().tag_map_) {
    if (current_tag.second == handle) {
      return current_tag.first;
    }
  }
  return tag;
}

// Class functions used for timing.
TimerImpl::TimerImpl(const std::string& tag, bool construct_stopped)
    : is_timing_(false), handle_(Timing::GetHandle(tag)), tag_(tag) {
  if (!construct_stopped) {
    Start();
  }
}

TimerImpl::~TimerImpl() {
  if (IsTiming()) {
    Stop();
  }
}

void TimerImpl::Start() {
  is_timing_ = true;
  time_ = std::chrono::system_clock::now();
}

double TimerImpl::Stop() {
  if (is_timing_) {
    std::chrono::time_point<std::chrono::system_clock> now =
        std::chrono::system_clock::now();
    double dt =
        static_cast<double>(
            std::chrono::duration_cast<std::chrono::nanoseconds>(now - time_)
                .count()) *
        kNumSecondsPerNanosecond;
    Timing::Instance().AddTime(handle_, dt);
    is_timing_ = false;
    return dt;
  }
  return 0.0;
}

void TimerImpl::Discard() {
  is_timing_ = false;
}

bool TimerImpl::IsTiming() const {
  return is_timing_;
}

size_t TimerImpl::GetHandle() const {
  return handle_;
}

void Timing::AddTime(size_t handle, double seconds) {
  std::lock_guard<std::mutex> lock(Instance().mutex_);
  timers_[handle].AddValue(seconds);
}

double Timing::GetTotalSeconds(size_t handle) {
  std::lock_guard<std::mutex> lock(Instance().mutex_);
  return Instance().timers_[handle].Sum();
}

double Timing::GetTotalSeconds(const std::string& tag) {
  return GetTotalSeconds(GetHandle(tag));
}

double Timing::GetMeanSeconds(size_t handle) {
  std::lock_guard<std::mutex> lock(Instance().mutex_);
  return Instance().timers_[handle].Mean();
}

double Timing::GetMeanSeconds(const std::string& tag) {
  return GetMeanSeconds(GetHandle(tag));
}

size_t Timing::GetNumSamples(size_t handle) {
  std::lock_guard<std::mutex> lock(Instance().mutex_);
  return Instance().timers_[handle].TotalSamples();
}

size_t Timing::GetNumSamples(const std::string& tag) {
  return GetNumSamples(GetHandle(tag));
}

double Timing::GetVarianceSeconds(size_t handle) {
  std::lock_guard<std::mutex> lock(Instance().mutex_);
  return Instance().timers_[handle].LazyVariance();
}

double Timing::GetVarianceSeconds(const std::string& tag) {
  return GetVarianceSeconds(GetHandle(tag));
}

double Timing::GetMinSeconds(size_t handle) {
  std::lock_guard<std::mutex> lock(Instance().mutex_);
  return Instance().timers_[handle].Min();
}

double Timing::GetMinSeconds(const std::string& tag) {
  return GetMinSeconds(GetHandle(tag));
}

double Timing::GetMaxSeconds(size_t handle) {
  std::lock_guard<std::mutex> lock(Instance().mutex_);
  return Instance().timers_[handle].Max();
}

double Timing::GetMaxSeconds(const std::string& tag) {
  return GetMaxSeconds(GetHandle(tag));
}

double Timing::GetHz(size_t handle) {
  std::lock_guard<std::mutex> lock(Instance().mutex_);
  return 1.0 / Instance().timers_[handle].RollingMean();
}

double Timing::GetHz(const std::string& tag) {
  return GetHz(GetHandle(tag));
}

std::string Timing::SecondsToTimeString(double seconds) {
  double secs = fmod(seconds, 60);
  int minutes = (seconds / 60);
  int hours = (seconds / 3600);
  minutes = minutes - (hours * 60);

  char buffer[256];
  snprintf(
      buffer, sizeof(buffer),
      "%02d:"
      "%02d:"
      "%09.6f",
      hours, minutes, secs);
  return buffer;
}

void Timing::WriteToYamlFile(const std::string& path) {
  const map_t& tag_map = Instance().tag_map_;

  if (tag_map.empty()) {
    return;
  }

  std::ofstream output_file(path);

  if (!output_file) {
    LOG(ERROR) << "Could not write timing: Unable to open file: " << path;
    return;
  }

  VLOG(1) << "Writing timing to file: " << path;
  for (const map_t::value_type& tag : tag_map) {
    const size_t index = tag.second;

    if (GetNumSamples(index) > 0) {
      std::string label = tag.first;

      // We do not want colons or hashes in a label, as they might interfere
      // with reading the yaml later.
      std::replace(label.begin(), label.end(), ':', '_');
      std::replace(label.begin(), label.end(), '#', '_');

      output_file << label << ":" << "\n";
      output_file << "  num_samples: " << GetNumSamples(index) << "\n";
      output_file << "  total: " << GetTotalSeconds(index) << "\n";
      output_file << "  mean: " << GetMeanSeconds(index) << "\n";
      output_file << "  std_dev: " << sqrt(GetVarianceSeconds(index)) << "\n";
      output_file << "  min: " << GetMinSeconds(index) << "\n";
      output_file << "  max: " << GetMaxSeconds(index) << "\n";
    }
    output_file << "\n";
  }
}

void Timing::Print(std::ostream& out) {  // NOLINT
  map_t tagMap;
  {
    std::lock_guard<std::mutex> lock(Instance().mutex_);
    tagMap = Instance().tag_map_;
  }

  if (tagMap.empty()) {
    return;
  }

  out << "SM Timing\n";
  out << "-----------\n";
  for (typename map_t::value_type t : tagMap) {
    size_t time_i = t.second;
    out.width((std::streamsize)Instance().max_tag_length_);
    out.setf(std::ios::left, std::ios::adjustfield);
    out << t.first << "\t";
    out.width(7);

    out.setf(std::ios::right, std::ios::adjustfield);
    out << GetNumSamples(time_i) << "\t";
    if (GetNumSamples(time_i) > 0) {
      out << SecondsToTimeString(GetTotalSeconds(time_i)) << "\t";
      double meansec = GetMeanSeconds(time_i);
      double stddev = sqrt(GetVarianceSeconds(time_i));
      out << "(" << SecondsToTimeString(meansec) << " +- ";
      out << SecondsToTimeString(stddev) << ")\t";

      double min_sec = GetMinSeconds(time_i);
      double max_sec = GetMaxSeconds(time_i);

      // The min or max are out of bounds.
      out << "[" << SecondsToTimeString(min_sec) << ","
          << SecondsToTimeString(max_sec) << "]";
    }
    out << std::endl;
  }
}

std::string Timing::Print() {
  std::stringstream ss;
  Print(ss);
  return ss.str();
}

void Timing::Reset() {
  std::lock_guard<std::mutex> lock(Instance().mutex_);
  Instance().tag_map_.clear();
}

}  // namespace timing
