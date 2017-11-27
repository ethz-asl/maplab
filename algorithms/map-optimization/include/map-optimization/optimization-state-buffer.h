#ifndef MAP_OPTIMIZATION_OPTIMIZATION_STATE_BUFFER_H_
#define MAP_OPTIMIZATION_OPTIMIZATION_STATE_BUFFER_H_

#include <unordered_map>
#include <vector>

#include <aslam/common/memory.h>
#include <glog/logging.h>
#include <vi-map/vi-map.h>

namespace map_optimization {

// Buffer for all states that can not be optimized directly on the map. For
// example the rotation of the keyframe pose is stored in a different
// convention than the optimization expects, hence, it is buffered here.
class OptimizationStateBuffer {
 public:
  void importStatesOfMissions(
      const vi_map::VIMap& map, const vi_map::MissionIdSet& mission_ids);
  void copyAllStatesBackToMap(vi_map::VIMap* map) const;

  double* get_vertex_q_IM__M_p_MI_JPL(const pose_graph::VertexId& id);
  double* get_baseframe_q_GM__G_p_GM_JPL(const vi_map::MissionBaseFrameId& id);
  double* get_camera_extrinsics_q_CI__C_p_CI_JPL(const aslam::CameraId& id);
  double* get_sensor_extrinsics_q_RS_JPL(const vi_map::SensorId& id);
  double* get_sensor_extrinsics_R_p_RS(const vi_map::SensorId& id);

 private:
  void importKeyframePosesOfMissions(
      const vi_map::VIMap& map, const vi_map::MissionIdSet& mission_ids);
  void importBaseframePoseOfMissions(
      const vi_map::VIMap& map, const vi_map::MissionIdSet& mission_ids);
  void importSensorCalibrationsOfMissions(
      const vi_map::VIMap& map, const vi_map::MissionIdSet& mission_ids);
  void importCameraCalibrationsOfMissions(
      const vi_map::VIMap& map, const vi_map::MissionIdSet& mission_ids);
  void copyAllKeyframePosesBackToMap(vi_map::VIMap* map) const;
  void copyAllBaseframePosesBackToMap(vi_map::VIMap* map) const;
  void copyAllSensorCalibrationsBackToMap(vi_map::VIMap* map) const;
  void copyAllCameraCalibrationsBackToMap(vi_map::VIMap* map) const;

  // Keyframe poses as a 7d vector: [q_IM_xyzw, M_p_MI]  (passive JPL).
  std::unordered_map<pose_graph::VertexId, size_t> vertex_id_to_vertex_idx_;
  Eigen::Matrix<double, 7, Eigen::Dynamic> vertex_q_IM__M_p_MI_;

  // Mission baseframe poses as a 7d vector: [q_IM_xyzw, M_p_MI] (passive JPL).
  std::unordered_map<vi_map::MissionBaseFrameId, size_t>
      baseframe_id_to_baseframe_idx_;
  Eigen::Matrix<double, 7, Eigen::Dynamic> baseframe_q_GM__G_p_GM_;

  // Camera extrinsics as 7d vector: [q_IC_xyzw, I_p_IC] (passive JPL).
  std::unordered_map<aslam::CameraId, std::vector<aslam::NCameraId>>
      camera_id_to_ncamera_id_;
  std::unordered_map<aslam::CameraId, size_t> camera_id_to_camera_idx_;
  Eigen::Matrix<double, 7, Eigen::Dynamic> camera_q_CI__C_p_CI_;

  // Optional sensor extrinsics (passive JPL). The sensor might only define
  // position, rotation or both at the same time.
  std::unordered_map<vi_map::SensorId, size_t> sensor_id_to_q_RS_idx_;
  Eigen::Matrix<double, 4, Eigen::Dynamic> sensor_q_RS_;
  std::unordered_map<vi_map::SensorId, size_t> sensor_id_to_R_p_RS__idx_;
  Eigen::Matrix<double, 3, Eigen::Dynamic> sensor_R_p_RS_;
};

inline void ensurePositiveQuaternion(Eigen::Ref<Eigen::Vector4d> quat_xyzw) {
  if (quat_xyzw(3) < 0.0) {
    quat_xyzw = -quat_xyzw;
  }
}

inline bool isValidQuaternion(const Eigen::Quaterniond& quat) {
  constexpr double kEpsilon = 1e-5;
  const double norm = quat.squaredNorm();
  return (quat.w() > 0.0) && (norm < ((1.0 + kEpsilon) * (1.0 + kEpsilon))) &&
         (norm > ((1.0 - kEpsilon) * (1.0 - kEpsilon)));
}

inline void assertValidQuaternion(const Eigen::Quaterniond& quat) {
  CHECK(isValidQuaternion(quat)) << "Quaternion: " << quat.coeffs().transpose()
                                 << " is not valid. (norm=" << quat.norm()
                                 << ")";
}
}  // namespace map_optimization
#endif  // MAP_OPTIMIZATION_OPTIMIZATION_STATE_BUFFER_H_
