#ifndef INTERNAL_VIO_TYPES_INL_H_
#define INTERNAL_VIO_TYPES_INL_H_

#include <algorithm>
#include <limits>
#include <string>
#include <unordered_map>

#include <aslam/frames/visual-frame.h>

namespace vio {

inline std::ostream& operator<<(
    std::ostream& out, const SynchronizedNFrameImu& value) {
  out << "\t nframe timestamps: ";
  for (size_t frame_idx = 0; frame_idx < value.nframe->getNumFrames();
       ++frame_idx) {
    if (frame_idx > 0) {
      out << ", ";
    }
    out << value.nframe->getFrame(frame_idx).getTimestampNanoseconds();
  }
  out << "\t imu timestamps: " << value.imu_timestamps << std::endl;
  return out;
}

inline std::ostream& operator<<(
    std::ostream& out, const EstimatorState& value) {
  static std::unordered_map<EstimatorState, std::string, EstimatorStateHash>
      names;
  if (names.size() == 0) {
#define INSERT_ELEMENT(type, val) names[type::val] = #val
    INSERT_ELEMENT(EstimatorState, kUninitialized);
    INSERT_ELEMENT(EstimatorState, kStartup);
    INSERT_ELEMENT(EstimatorState, kRunning);
#undef INSERT_ELEMENT
  }
  return out << names[value];
}

inline std::ostream& operator<<(std::ostream& out, const MotionType& value) {
  static std::unordered_map<MotionType, std::string, MotionTypeHash> names;
  if (names.size() == 0) {
#define INSERT_ELEMENT(type, val) names[type::val] = #val
    INSERT_ELEMENT(MotionType, kInvalid);
    INSERT_ELEMENT(MotionType, kRotationOnly);
    INSERT_ELEMENT(MotionType, kGeneralMotion);
#undef INSERT_ELEMENT
  }
  return out << names[value];
}

inline std::ostream& operator<<(
    std::ostream& out, const LocalizationState& value) {
  static std::unordered_map<LocalizationState, std::string,
                            LocalizationStateHash>
      names;
  if (names.size() == 0u) {
#define INSERT_ELEMENT(type, val) names[type::val] = #val
    INSERT_ELEMENT(LocalizationState, kUninitialized);
    INSERT_ELEMENT(LocalizationState, kNotLocalized);
    INSERT_ELEMENT(LocalizationState, kLocalized);
    INSERT_ELEMENT(LocalizationState, kMapTracking);
#undef INSERT_ELEMENT
  }
  return out << names[value];
}

inline std::ostream& operator<<(std::ostream& out, const ViNodeState& value) {
  out << "\tq_M_I: " << value.get_T_M_I().getRotation().vector().transpose()
      << std::endl;
  out << "\tT_M_I: " << value.get_T_M_I().getPosition().transpose()
      << std::endl;
  out << "\tv_M_I: " << value.get_v_M_I().transpose() << std::endl;
  out << "\tacc_bias: " << value.getAccBias().transpose() << std::endl;
  out << "\tgyro_bias: " << value.getGyroBias().transpose() << std::endl;
  return out;
}

inline std::ostream& operator<<(
    std::ostream& out, const NFrameIdViNodeStatePair& value) {
  out << "ViNodeState @ NFrameId =" << value.first << std::endl;
  out << value.second;
  return out;
}

inline std::ostream& operator<<(std::ostream& out, const ViNodeStates& pairs) {
  for (const NFrameIdViNodeStatePair& pair : pairs) {
    out << pair;
  }
  return out;
}

inline std::ostream& operator<<(
    std::ostream& out, const NFrameIdViNodeStateMap& map) {
  for (const NFrameIdViNodeStatePair& pair : map) {
    out << pair;
  }
  return out;
}

}  // namespace vio

#endif  // INTERNAL_VIO_TYPES_INL_H_
