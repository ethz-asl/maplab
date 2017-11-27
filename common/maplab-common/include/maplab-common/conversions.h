#ifndef MAPLAB_COMMON_CONVERSIONS_H_
#define MAPLAB_COMMON_CONVERSIONS_H_

#define _USE_MATH_DEFINES
#include <cmath>
#include <cstddef>
#include <cstdint>

// Please use the namespaces below when adding new constants.

constexpr double kMilliSecondsToSeconds = 1e-3;
constexpr double kSecondsToMilliSeconds = 1e3;
constexpr int64_t kMicrosecondsToNanoseconds = 1000;

constexpr double kMillisecondsToMicroseconds = 1e3;

constexpr double kNanosecondsToSeconds = 1e-9;
constexpr double kSecondsToNanoSeconds = 1e9;

constexpr double kRadToDeg = 180.0 / M_PI;
constexpr double kDegToRad = M_PI / 180.0;

namespace common {
namespace conversions {

constexpr size_t kBitsPerByte = 8u;

constexpr size_t kMilliSecondsToNanoSeconds = 1e6;

}  // namespace conversions
}  // namespace common

#endif  // MAPLAB_COMMON_CONVERSIONS_H_
