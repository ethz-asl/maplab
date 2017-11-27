#ifndef MAP_OPTIMIZATION_MISSION_CLUSTER_GAUGE_FIXES_INL_H_
#define MAP_OPTIMIZATION_MISSION_CLUSTER_GAUGE_FIXES_INL_H_

#include <algorithm>
#include <string>

#include <vi-map/unique-id.h>

namespace map_optimization {
inline std::string FixedRotationDoFToString(const FixedRotationDoF& value) {
  switch (value) {
    case FixedRotationDoF::kNone:
      return "kNone";
    case FixedRotationDoF::kYaw:
      return "kYaw";
    case FixedRotationDoF::kAll:
      return "kAll";
    default:
      LOG(FATAL) << "Unhandled value.";
  }
  // Suppress warning.
  return "";
}

inline std::ostream& operator<<(
    std::ostream& out, const FixedRotationDoF& value) {
  return out << FixedRotationDoFToString(value);
}
}  // namespace map_optimization

#endif  // MAP_OPTIMIZATION_MISSION_CLUSTER_GAUGE_FIXES_INL_H_
