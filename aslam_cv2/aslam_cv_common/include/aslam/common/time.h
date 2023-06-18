#ifndef ASLAM_TIME_H_
#define ASLAM_TIME_H_

#include <chrono>
#include <cstdint>

namespace aslam {
namespace time {

namespace internal {
template<typename TimeUnit> struct time_traits;
struct sec; template<> struct time_traits<sec> { static constexpr int64_t nanoseconds = 1e9; };
struct milli; template<> struct time_traits<milli> { static constexpr int64_t nanoseconds = 1e6; };
struct micro; template<> struct time_traits<micro> { static constexpr int64_t nanoseconds = 1e3; };
struct nano; template<> struct time_traits<nano> { static constexpr int64_t nanoseconds = 1; };
template<typename TimeUnit> inline constexpr int64_t convertToNanosecondsCompileTime(
    int64_t value) {
  return value * time_traits<TimeUnit>::nanoseconds;
}
template<typename TimeUnit> inline static int64_t convertToNanoseconds(double value) {
  return static_cast<int64_t>(value * time_traits<TimeUnit>::nanoseconds);
}
template<typename TimeUnit> inline static double convertFromNanoseconds(int64_t value) {
  return static_cast<double>(value) / static_cast<double>(time_traits<TimeUnit>::nanoseconds);
}
}  // namespace internal

/// Convenience functions to convert the specified unit to the nanoseconds format at compile time.
/// Example: int64_t sampling_time = aslam::time::microseconds(10);
constexpr auto seconds = internal::convertToNanosecondsCompileTime<internal::sec>;
constexpr auto milliseconds = internal::convertToNanosecondsCompileTime<internal::milli>;
constexpr auto microseconds = internal::convertToNanosecondsCompileTime<internal::micro>;
constexpr auto nanoseconds = internal::convertToNanosecondsCompileTime<internal::nano>;

/// Convert a timestamp (in nanoseconds) to other time units at runtime.
/// Example: double milliseconds = aslam::time::to_milliseconds(int64_t timestamp_nanoseconds);
#define FUNC_ALIAS __attribute__((unused)) static auto
FUNC_ALIAS to_seconds = internal::convertFromNanoseconds<internal::sec>;
FUNC_ALIAS to_milliseconds = internal::convertFromNanoseconds<internal::milli>;
FUNC_ALIAS to_microseconds = internal::convertFromNanoseconds<internal::micro>;

/// Convert other time units to a timestamp (in nanoseconds) at runtime.
/// Example: int64_t timestamp_nanoseconds = aslam::time::from_seconds(double seconds);
FUNC_ALIAS from_seconds = internal::convertToNanoseconds<internal::sec>;
FUNC_ALIAS from_milliseconds = internal::convertToNanoseconds<internal::milli>;
FUNC_ALIAS from_microseconds = internal::convertToNanoseconds<internal::micro>;

/// \brief get the current time in nanoseconds since epoch.
inline int64_t nanoSecondsSinceEpoch() {
    return std::chrono::duration_cast<std::chrono::nanoseconds>
        (std::chrono::system_clock::now().time_since_epoch()).count();
}

/// \brief convert double seconds to integer nanoseconds since epoch.
inline int64_t secondsToNanoSeconds(double seconds) {
  return from_seconds(seconds);
}

/// \brief convert integer nanoseconds into double seconds since epoch.
inline double nanoSecondsToSeconds(int64_t nano_seconds) {
  return to_seconds(nano_seconds);
}

/// \brief return a magic number representing an invalid timestamp.
///        std::numeric_limits<int64_t>::min()
inline constexpr int64_t getInvalidTime() {
  return std::numeric_limits<int64_t>::min();
}

/// \brief Is the time valid? This uses a magic number
inline bool isValidTime(int64_t time) {
  return time != getInvalidTime();
}

inline std::string timeNanosecondsToString(const int64_t timestamp_nanoseconds) {
  const std::string timestamp_string = std::to_string(timestamp_nanoseconds);
  const size_t num_digits = timestamp_string.size();
  if (num_digits < 10u) {
    return timestamp_string + "ns";
  }

  const std::string timestamp_with_units =
      timestamp_string.substr(0u, num_digits - 9u) + "s " +
      timestamp_string.substr(num_digits - 9u) + "ns";
  return timestamp_with_units;
}

}  // namespace time
}  // namespace aslam

#endif  // ASLAM_TIME_H_
