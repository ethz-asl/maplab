#include "map-optimization/optimization-state-buffer.h"

#include <unordered_set>
#include <vector>

#include <Eigen/Core>
#include <aslam/common/unique-id.h>
#include <glog/logging.h>
#include <maplab-common/accessors.h>
#include <vi-map/sensor-manager.h>

namespace map_optimization {

double* OptimizationStateBuffer::get_vertex_q_IM__M_p_MI_JPL(
    const pose_graph::VertexId& id) {
  const size_t index = common::getChecked(vertex_id_to_vertex_idx_, id);
  CHECK_LT(index, static_cast<size_t>(vertex_q_IM__M_p_MI_.cols()));
  return vertex_q_IM__M_p_MI_.col(index).data();
}

double* OptimizationStateBuffer::get_baseframe_q_GM__G_p_GM_JPL(
    const vi_map::MissionBaseFrameId& id) {
  const size_t index = common::getChecked(baseframe_id_to_baseframe_idx_, id);
  CHECK_LT(index, static_cast<size_t>(baseframe_q_GM__G_p_GM_.cols()));
  return baseframe_q_GM__G_p_GM_.col(index).data();
}

double* OptimizationStateBuffer::get_camera_extrinsics_q_CI__C_p_CI_JPL(
    const aslam::CameraId& id) {
  const size_t index = common::getChecked(camera_id_to_camera_idx_, id);
  CHECK_LT(index, static_cast<size_t>(camera_q_CI__C_p_CI_.cols()));
  return camera_q_CI__C_p_CI_.col(index).data();
}

double* OptimizationStateBuffer::get_sensor_extrinsics_q_SB__S_p_SB_JPL(
    const aslam::SensorId& id) {
  const size_t index = common::getChecked(other_sensor_id_to_sensor_idx_, id);
  CHECK_LT(index, static_cast<size_t>(other_sensor_q_SB__S_p_SB_.cols()));
  return other_sensor_q_SB__S_p_SB_.col(index).data();
}

void OptimizationStateBuffer::copyAllStatesBackToMap(vi_map::VIMap* map) const {
  copyAllKeyframePosesBackToMap(map);
  copyAllBaseframePosesBackToMap(map);
  copyAllCameraCalibrationsBackToMap(map);
  copyAllOtherSensorExtrinsicsBackToMap(map);
}

void OptimizationStateBuffer::copyAllKeyframePosesBackToMap(
    vi_map::VIMap* map) const {
  CHECK_NOTNULL(map);
  CHECK_EQ(
      static_cast<size_t>(vertex_q_IM__M_p_MI_.cols()),
      vertex_id_to_vertex_idx_.size());
  typedef std::pair<const pose_graph::VertexId, size_t> value_type;
  for (const value_type& vertex_id_idx : vertex_id_to_vertex_idx_) {
    vi_map::Vertex& vertex = map->getVertex(vertex_id_idx.first);
    const size_t vertex_idx = vertex_id_idx.second;
    Eigen::Map<Eigen::Quaterniond> map_q_M_I(vertex.get_q_M_I_Mutable());
    Eigen::Map<Eigen::Vector3d> map_p_M_I(vertex.get_p_M_I_Mutable());

    CHECK_LT(vertex_idx, static_cast<size_t>(vertex_q_IM__M_p_MI_.cols()));

    // Change from JPL passive quaternion used by error terms to active Hamilton
    // quaternion.
    Eigen::Quaterniond q_I_M_JPL;
    q_I_M_JPL.coeffs() = vertex_q_IM__M_p_MI_.col(vertex_idx).head<4>();
    assertValidQuaternion(q_I_M_JPL);

    // I_q_G_JPL is in fact equal to active G_q_I - no inverse is needed.
    map_q_M_I = q_I_M_JPL;
    map_p_M_I = vertex_q_IM__M_p_MI_.col(vertex_idx).tail<3>();
  }
}

void OptimizationStateBuffer::copyAllBaseframePosesBackToMap(
    vi_map::VIMap* map) const {
  CHECK_NOTNULL(map);

  typedef std::pair<const vi_map::MissionBaseFrameId, size_t> value_type;
  for (const value_type& baseframe_id_idx : baseframe_id_to_baseframe_idx_) {
    vi_map::MissionBaseFrame& baseframe =
        map->getMissionBaseFrame(baseframe_id_idx.first);
    const size_t baseframe_idx = baseframe_id_idx.second;
    CHECK_LT(baseframe_idx, static_cast<size_t>(vertex_q_IM__M_p_MI_.cols()));

    // Change from JPL passive quaternion used by error terms to active
    // quaternion in the system. Inverse needed.
    Eigen::Quaterniond q_G_M_JPL(
        baseframe_q_GM__G_p_GM_.col(baseframe_idx).head<4>().data());
    assertValidQuaternion(q_G_M_JPL);
    baseframe.set_q_G_M(q_G_M_JPL.inverse());
    baseframe.set_p_G_M(baseframe_q_GM__G_p_GM_.col(baseframe_idx).tail<3>());
  }
}

void OptimizationStateBuffer::copyAllCameraCalibrationsBackToMap(
    vi_map::VIMap* map) const {
  CHECK_NOTNULL(map);

  vi_map::SensorManager& sensor_manager = map->getSensorManager();

  for (const std::pair<const aslam::CameraId, std::vector<aslam::NCameraId>>&
           camid_ncamids : camera_id_to_ncamera_id_) {
    for (const aslam::NCameraId& ncamid : camid_ncamids.second) {
      aslam::NCamera::Ptr ncamera =
          sensor_manager.getSensorPtr<aslam::NCamera>(ncamid);
      CHECK(ncamera);
      const size_t index =
          common::getChecked(camera_id_to_camera_idx_, camid_ncamids.first);

      Eigen::Quaterniond q_C_I_JPL(
          camera_q_CI__C_p_CI_.col(index).head<4>().data());
      assertValidQuaternion(q_C_I_JPL);

      // Change from JPL passive quaternion used by error terms to active
      // quaternion in the system. Inverse needed.
      aslam::Transformation T_C_I(
          camera_q_CI__C_p_CI_.col(index).tail<3>(), q_C_I_JPL.inverse());

      const int camera_idx_in_ncamera =
          ncamera->getCameraIndex(camid_ncamids.first);
      CHECK_GE(camera_idx_in_ncamera, 0);
      ncamera->set_T_C_B(static_cast<size_t>(camera_idx_in_ncamera), T_C_I);
    }
  }
}

void OptimizationStateBuffer::copyAllOtherSensorExtrinsicsBackToMap(
    vi_map::VIMap* map) const {
  CHECK_NOTNULL(map);

  vi_map::SensorManager& sensor_manager = map->getSensorManager();

  for (const std::pair<const aslam::SensorId, size_t>& sensor_id_to_idx :
       other_sensor_id_to_sensor_idx_) {
    Eigen::Quaterniond q_S_B_JPL(
        other_sensor_q_SB__S_p_SB_.col(sensor_id_to_idx.second)
            .head<4>()
            .data());
    assertValidQuaternion(q_S_B_JPL);

    // Change from JPL passive quaternion used by error terms to active
    // quaternion in the system. Inverse needed.
    aslam::Transformation T_S_B(
        other_sensor_q_SB__S_p_SB_.col(sensor_id_to_idx.second).tail<3>(),
        q_S_B_JPL.inverse());

    sensor_manager.setSensor_T_B_S(sensor_id_to_idx.first, T_S_B.inverse());
  }
}

void OptimizationStateBuffer::importStatesOfMissions(
    const vi_map::VIMap& map, const vi_map::MissionIdSet& mission_ids) {
  importKeyframePosesOfMissions(map, mission_ids);
  importBaseframePoseOfMissions(map, mission_ids);
  importCameraCalibrationsOfMissions(map, mission_ids);
  importOtherSensorExtrinsicsOfMissions(map, mission_ids);
}

void OptimizationStateBuffer::importKeyframePosesOfMissions(
    const vi_map::VIMap& map, const vi_map::MissionIdSet& mission_ids) {
  pose_graph::VertexIdList all_vertices;
  pose_graph::VertexIdList mission_vertices;
  for (const vi_map::MissionId& mission_id : mission_ids) {
    map.getAllVertexIdsInMissionAlongGraph(mission_id, &mission_vertices);
    all_vertices.insert(
        all_vertices.end(), mission_vertices.begin(), mission_vertices.end());
  }
  vertex_id_to_vertex_idx_.reserve(all_vertices.size());
  vertex_q_IM__M_p_MI_.resize(Eigen::NoChange, all_vertices.size());

  size_t vertex_idx = 0u;
  for (const pose_graph::VertexId& vertex_id : all_vertices) {
    const vi_map::Vertex& ba_vertex = map.getVertex(vertex_id);

    Eigen::Quaterniond q_M_I = ba_vertex.get_q_M_I();
    ensurePositiveQuaternion(q_M_I.coeffs());
    assertValidQuaternion(q_M_I);

    // Convert active Hamiltonian rotation (minkindr, Eigen) to passive JPL
    // which the error terms use. No inverse is required.
    vertex_q_IM__M_p_MI_.col(vertex_idx) << q_M_I.coeffs(),
        ba_vertex.get_p_M_I();
    CHECK(ba_vertex.id().isValid());
    CHECK(vertex_id_to_vertex_idx_.emplace(ba_vertex.id(), vertex_idx).second);
    ++vertex_idx;
  }
  CHECK_EQ(
      static_cast<size_t>(vertex_q_IM__M_p_MI_.cols()),
      vertex_id_to_vertex_idx_.size());
  CHECK_EQ(all_vertices.size(), vertex_id_to_vertex_idx_.size());
}

void OptimizationStateBuffer::importBaseframePoseOfMissions(
    const vi_map::VIMap& map, const vi_map::MissionIdSet& mission_ids) {
  baseframe_q_GM__G_p_GM_.resize(Eigen::NoChange, mission_ids.size());
  size_t base_frame_idx = 0u;
  for (const vi_map::MissionId& mission_id : mission_ids) {
    const vi_map::MissionBaseFrame& baseframe =
        map.getMissionBaseFrameForMission(mission_id);

    // Convert active Hamiltonian rotation (minkindr, Eigen) to passive JPL
    // which the error terms use. An inverse is required.
    Eigen::Quaterniond q_G_M_JPL(baseframe.get_q_G_M().inverse().coeffs());
    ensurePositiveQuaternion(q_G_M_JPL.coeffs());
    assertValidQuaternion(q_G_M_JPL);
    baseframe_q_GM__G_p_GM_.col(base_frame_idx) << q_G_M_JPL.coeffs(),
        baseframe.get_p_G_M();

    CHECK(baseframe.id().isValid());
    CHECK(baseframe_id_to_baseframe_idx_.emplace(baseframe.id(), base_frame_idx)
              .second);
    ++base_frame_idx;
  }

  CHECK_EQ(
      static_cast<size_t>(baseframe_q_GM__G_p_GM_.cols()),
      baseframe_id_to_baseframe_idx_.size());
  CHECK_EQ(mission_ids.size(), baseframe_id_to_baseframe_idx_.size());
}

void OptimizationStateBuffer::importCameraCalibrationsOfMissions(
    const vi_map::VIMap& map, const vi_map::MissionIdSet& mission_ids) {
  size_t num_cameras = 0u;
  std::vector<const aslam::NCamera*> ncameras;
  for (const vi_map::MissionId& mission_id : mission_ids) {
    if (map.getMission(mission_id).hasNCamera()) {
      const aslam::NCamera& ncamera = map.getMissionNCamera(mission_id);
      num_cameras += ncamera.getNumCameras();
      ncameras.emplace_back(&ncamera);
    }
  }
  camera_q_CI__C_p_CI_.resize(Eigen::NoChange, num_cameras);

  size_t col_idx = 0u;
  for (const aslam::NCamera* const ncamera_ptr : ncameras) {
    const aslam::NCamera& ncamera = *CHECK_NOTNULL(ncamera_ptr);
    for (size_t cam_idx = 0u; cam_idx < ncamera.numCameras(); ++cam_idx) {
      const aslam::Transformation& T_C_I = ncamera.get_T_C_B(cam_idx);

      // Convert active Hamiltonian rotation (minkindr, Eigen) to passive JPL
      // which the error terms use. An inverse is required.
      Eigen::Quaterniond q_C_I_JPL(
          T_C_I.getRotation().toImplementation().inverse().coeffs());
      ensurePositiveQuaternion(q_C_I_JPL.coeffs());
      assertValidQuaternion(q_C_I_JPL);
      camera_q_CI__C_p_CI_.col(col_idx) << q_C_I_JPL.coeffs(),
          T_C_I.getPosition();

      const aslam::CameraId& camera_id = ncamera.getCamera(cam_idx).getId();
      CHECK(camera_id.isValid());
      camera_id_to_ncamera_id_[camera_id].emplace_back(ncamera.getId());

      // The same camera can be assigned to multiple missions. Therefore we
      // won't check the success of insertion.
      camera_id_to_camera_idx_.emplace(camera_id, col_idx);
      ++col_idx;
    }
  }
}

void OptimizationStateBuffer::importOtherSensorExtrinsicsOfMissions(
    const vi_map::VIMap& map, const vi_map::MissionIdSet& mission_ids) {
  std::vector<aslam::SensorId> other_sensor_ids;
  for (const vi_map::MissionId& mission_id : mission_ids) {
    const vi_map::VIMission& mission = map.getMission(mission_id);
    if (mission.hasAbsolute6DoFSensor()) {
      other_sensor_ids.emplace_back(
          mission.getAbsolute6DoFSensor());
    }

    if (mission.hasOdometry6DoFSensor()) {
      other_sensor_ids.emplace_back(mission.getOdometry6DoFSensor());
    }

    if (mission.hasWheelOdometrySensor()) {
      other_sensor_ids.emplace_back(mission.getWheelOdometrySensor());
    }
  }
  LOG(INFO) << "Added " << other_sensor_ids.size() << " sensors.";

  other_sensor_q_SB__S_p_SB_.resize(Eigen::NoChange, other_sensor_ids.size());

  size_t col_idx = 0u;
  for (const aslam::SensorId& sensor_id : other_sensor_ids) {
    const aslam::Transformation& T_S_B =
        map.getSensorManager().getSensor_T_B_S(sensor_id).inverse();

    // Convert active Hamiltonian rotation (minkindr, Eigen) to passive JPL
    // which the error terms use. An inverse is required.
    Eigen::Quaterniond q_S_B_JPL(
        T_S_B.getRotation().toImplementation().inverse().coeffs());
    ensurePositiveQuaternion(q_S_B_JPL.coeffs());
    assertValidQuaternion(q_S_B_JPL);
    other_sensor_q_SB__S_p_SB_.col(col_idx) << q_S_B_JPL.coeffs(),
        T_S_B.getPosition();

    other_sensor_id_to_sensor_idx_.emplace(sensor_id, col_idx);
    ++col_idx;
  }
}

}  // namespace map_optimization
