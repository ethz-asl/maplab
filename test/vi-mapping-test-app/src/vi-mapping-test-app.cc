#include "vi-mapping-test-app/vi-mapping-test-app.h"

#include <random>

#include <aslam/tracker/tracking-helpers.h>
#include <gtest/gtest.h>
#include <map-manager/map-manager.h>
#include <maplab-common/test/testing-predicates.h>
#include <vi-map/check-map-consistency.h>
#include <vi-map/vi-map.h>

namespace visual_inertial_mapping {

VIMappingTestApp::VIMappingTestApp() : random_seed_(5) {}

VIMappingTestApp::~VIMappingTestApp() {
  vi_map::VIMapManager map_manager;
  if (map_manager.hasMap(map_key_)) {
    map_manager.deleteMap(map_key_);
  }
}

void VIMappingTestApp::loadDataset(const std::string& folder_name) {
  // Store the reference keyframe poses and landmark positions.
  vi_map::VIMapManager map_manager;
  map_manager.loadMapFromFolder(folder_name, &map_key_);
  vi_map::VIMapManager::MapWriteAccess map =
      map_manager.getMapWriteAccess(map_key_);

  vi_map::LandmarkIdSet landmark_ids;
  map->getAllLandmarkIds(&landmark_ids);
  for (const vi_map::LandmarkId& landmark_id : landmark_ids) {
    const Eigen::Vector3d& p_B_fi = map->getLandmark(landmark_id).get_p_B();
    landmark_reference_positions_.insert(std::make_pair(landmark_id, p_B_fi));
  }

  pose_graph::VertexIdList vertex_ids;
  map->getAllVertexIds(&vertex_ids);
  for (const pose_graph::VertexId& vertex_id : vertex_ids) {
    const pose::Transformation& T_M_I = map->getVertex(vertex_id).get_T_M_I();
    vertex_reference_poses_.insert(std::make_pair(vertex_id, T_M_I));
  }
}

vi_map::VIMap* VIMappingTestApp::getMapMutable() {
  vi_map::VIMapManager map_manager;
  vi_map::VIMapManager::MapWriteAccess map =
      map_manager.getMapWriteAccess(map_key_);
  return map.get();
}

bool VIMappingTestApp::isMapConsistent() {
  const vi_map::VIMapManager map_manager;
  vi_map::VIMapManager::MapReadAccess map =
      map_manager.getMapReadAccess(map_key_);
  return vi_map::checkMapConsistency(*map);
}

void VIMappingTestApp::sparsifyMission() {
  vi_map::VIMapManager map_manager;
  vi_map::VIMapManager::MapWriteAccess map =
      map_manager.getMapWriteAccess(map_key_);
  vi_map::MissionIdList mission_ids;
  map->getAllMissionIds(&mission_ids);
  CHECK(!mission_ids.empty());
  const vi_map::MissionId& first_mission_id = *mission_ids.begin();

  constexpr int kEveryNthVertexToKeep = 5;
  map->sparsifyMission(first_mission_id, kEveryNthVertexToKeep);
}

size_t VIMappingTestApp::numVerticesOnMap() const {
  const vi_map::VIMapManager map_manager;
  vi_map::VIMapManager::MapReadAccess map =
      map_manager.getMapReadAccess(map_key_);
  return map->numVertices();
}

void VIMappingTestApp::corruptLandmarkPositions(
    double std_dev_m, int every_nth) {
  CHECK_GE(std_dev_m, 0);
  std::mt19937 gen(random_seed_);
  std::normal_distribution<> dis(0, std_dev_m);

  vi_map::VIMapManager map_manager;
  vi_map::VIMapManager::MapWriteAccess map =
      map_manager.getMapWriteAccess(map_key_);

  vi_map::LandmarkIdSet landmark_ids;
  map->getAllLandmarkIds(&landmark_ids);
  unsigned int index = 0;
  for (const vi_map::LandmarkId& landmark_id : landmark_ids) {
    if (index % every_nth == 0) {
      vi_map::Landmark& landmark = map->getLandmark(landmark_id);
      const Eigen::Vector3d& p_B_fi = landmark.get_p_B();
      const Eigen::Vector3d noise(dis(gen), dis(gen), dis(gen));
      landmark.set_p_B(p_B_fi + noise);
    }

    ++index;
  }
}

void VIMappingTestApp::corruptKeyframePoses(
    double position_std_dev_m, double orientation_std_dev_quat, int every_nth) {
  CHECK_GE(position_std_dev_m, 0);
  CHECK_GE(orientation_std_dev_quat, 0);
  std::mt19937 gen(random_seed_);
  std::normal_distribution<> p_dis(0, position_std_dev_m);
  std::normal_distribution<> q_dis(0, orientation_std_dev_quat);

  vi_map::VIMapManager map_manager;
  vi_map::VIMapManager::MapWriteAccess map =
      map_manager.getMapWriteAccess(map_key_);

  // Get set of first two vertices that shouldn't be corrupted.
  vi_map::MissionIdList mission_ids;
  map->getAllMissionIds(&mission_ids);
  pose_graph::VertexIdSet not_to_corrupt_vertices;
  for (const vi_map::MissionId& mission_id : mission_ids) {
    const vi_map::Mission& mission = map->getMission(mission_id);
    pose_graph::VertexId current_vertex_id = mission.getRootVertexId();
    not_to_corrupt_vertices.insert(current_vertex_id);

    if (map->getNextVertex(
            current_vertex_id,
            &current_vertex_id)) {
      not_to_corrupt_vertices.insert(current_vertex_id);
    }
  }

  pose_graph::VertexIdList vertex_ids;
  unsigned int index = 0;
  map->getAllVertexIds(&vertex_ids);
  for (const pose_graph::VertexId& vertex_id : vertex_ids) {
    // This vertex will be fixed by the optimizer, do not perturb it.
    if (not_to_corrupt_vertices.count(vertex_id) > 0) {
      continue;
    }

    if (index % every_nth == 0) {
      vi_map::Vertex& vertex = map->getVertex(vertex_id);
      pose::Transformation T_M_I = vertex.get_T_M_I();

      const Eigen::Vector3d p_noise(p_dis(gen), p_dis(gen), p_dis(gen));
      T_M_I.getPosition() += p_noise;

      Eigen::Quaterniond q_noise(1, q_dis(gen), q_dis(gen), q_dis(gen));
      q_noise.normalize();
      T_M_I.getRotation().toImplementation() *= q_noise;

      vertex.set_T_M_I(T_M_I);
    }

    ++index;
  }
}

void VIMappingTestApp::corruptAbs6DoFSensorExtrinsics(
    double position_std_dev_m, double orientation_std_dev_quat) {
  CHECK_GE(position_std_dev_m, 0);
  CHECK_GE(orientation_std_dev_quat, 0);
  std::mt19937 gen(random_seed_);
  std::normal_distribution<> p_dis(0, position_std_dev_m);
  std::normal_distribution<> q_dis(0, orientation_std_dev_quat);

  vi_map::VIMapManager map_manager;
  vi_map::VIMapManager::MapWriteAccess map =
      map_manager.getMapWriteAccess(map_key_);

  // Get set of first two vertices that shouldn't be corrupted.
  vi_map::MissionIdList mission_ids;
  map->getAllMissionIds(&mission_ids);
  pose_graph::VertexIdSet not_to_corrupt_vertices;
  for (const vi_map::MissionId& mission_id : mission_ids) {
    vi_map::VIMission& mission = map->getMission(mission_id);

    if (!mission.hasAbsolute6DoFSensor()) {
      continue;
    }

    const aslam::SensorId& sensor_id = mission.getAbsolute6DoFSensor();

    aslam::Transformation& T_B_S =
        map->getSensorManager().getSensor_T_B_S(sensor_id);

    const Eigen::Vector3d p_noise(p_dis(gen), p_dis(gen), p_dis(gen));
    T_B_S.getPosition() += p_noise;

    Eigen::Quaterniond q_noise(1, q_dis(gen), q_dis(gen), q_dis(gen));
    q_noise.normalize();
    T_B_S.getRotation().toImplementation() *= q_noise;
  }
}

void VIMappingTestApp::corruptCameraExtrinsics(
    double position_std_dev_m, double orientation_std_dev_quat) {
  CHECK_GE(position_std_dev_m, 0);
  CHECK_GE(orientation_std_dev_quat, 0);
  std::mt19937 gen(random_seed_);
  std::normal_distribution<> p_dis(0, position_std_dev_m);
  std::normal_distribution<> q_dis(0, orientation_std_dev_quat);

  vi_map::VIMapManager map_manager;
  vi_map::VIMapManager::MapWriteAccess map =
      map_manager.getMapWriteAccess(map_key_);

  // Get set of first two vertices that shouldn't be corrupted.
  vi_map::MissionIdList mission_ids;
  map->getAllMissionIds(&mission_ids);
  pose_graph::VertexIdSet not_to_corrupt_vertices;
  for (const vi_map::MissionId& mission_id : mission_ids) {
    vi_map::VIMission& mission = map->getMission(mission_id);

    if (!mission.hasNCamera()) {
      continue;
    }

    const aslam::SensorId& sensor_id = mission.getNCameraId();

    aslam::NCamera::Ptr ncamera =
        map->getSensorManager().getSensorPtr<aslam::NCamera>(sensor_id);
    CHECK(ncamera);

    for (size_t camera_idx = 0u; camera_idx < ncamera->getNumCameras();
         ++camera_idx) {
      aslam::Transformation& T_C_B = ncamera->get_T_C_B_Mutable(camera_idx);

      const Eigen::Vector3d p_noise(p_dis(gen), p_dis(gen), p_dis(gen));
      T_C_B.getPosition() += p_noise;

      Eigen::Quaterniond q_noise(1, q_dis(gen), q_dis(gen), q_dis(gen));
      q_noise.normalize();
      T_C_B.getRotation().toImplementation() *= q_noise;
    }
  }
}

void VIMappingTestApp::addCorruptDuplicateLandmarkObservations(int every_nth) {
  vi_map::VIMapManager map_manager;
  vi_map::VIMapManager::MapWriteAccess map =
      map_manager.getMapWriteAccess(map_key_);

  vi_map::LandmarkIdList landmark_ids;
  map->getAllLandmarkIds(&landmark_ids);

  size_t index = 0;

  for (const vi_map::LandmarkId& landmark_id : landmark_ids) {
    if (!landmark_id.isValid()) {
      continue;
    }
    vi_map::Landmark& landmark = map->getLandmark(landmark_id);
    const vi_map::KeypointIdentifierList& landmark_observations =
        landmark.getObservations();
    for (size_t idx = 0u; idx < landmark_observations.size(); ++idx) {
      const vi_map::KeypointIdentifier& keypoint = landmark_observations[idx];
      const pose_graph::VertexId& vertex_id = keypoint.frame_id.vertex_id;
      if (index % every_nth != 0 || !keypoint.isValid() ||
          !vertex_id.isValid()) {
        ++index;
        continue;
      }

      // Search for the observation with the largest pixel distance from the
      // keypoint in this frame.
      vi_map::Vertex& vertex = map->getVertex(vertex_id);
      const aslam::VisualFrame& frame =
          vertex.getVisualFrame(keypoint.frame_id.frame_index);
      const Eigen::Vector2d& keypoint_measurement =
          frame.getKeypointMeasurement(keypoint.keypoint_index);
      double max_distance = 0.0;
      size_t max_distance_idx;
      for (size_t measurement_idx = 0u;
           measurement_idx < frame.getNumKeypointMeasurements();
           ++measurement_idx) {
        const Eigen::Vector2d& other_measurement =
            frame.getKeypointMeasurement(measurement_idx);
        const double distance =
            (keypoint_measurement - other_measurement).norm();
        if (distance > max_distance) {
          max_distance = distance;
          max_distance_idx = measurement_idx;
        }
      }

      const vi_map::LandmarkId& max_distance_landmark_id =
          vertex.getObservedLandmarkId(
              keypoint.frame_id.frame_index, max_distance_idx);

      // No work to do here.
      if (max_distance_landmark_id == landmark_id) {
        continue;
      }

      // Set the maximum distance observation to the same landmark and do
      // the bookkeeping.
      vi_map::KeypointIdentifier max_distance_keypoint = keypoint;
      max_distance_keypoint.keypoint_index = max_distance_idx;
      if (max_distance_landmark_id.isValid()) {
        const pose_graph::VertexId& storing_vertex_id =
            map->getLandmarkStoreVertexId(max_distance_landmark_id);
        // In this case we skip for simplicity, so we don't have to rearrange
        // the landmark store.
        if (vertex_id == storing_vertex_id) {
          continue;
        }
        vi_map::Landmark& max_distance_landmark =
            map->getLandmark(max_distance_landmark_id);
        max_distance_landmark.removeObservation(max_distance_keypoint);
      }
      vertex.setObservedLandmarkId(max_distance_keypoint, landmark_id);
      landmark.addObservation(max_distance_keypoint);
      ++index;
    }
  }
}

void VIMappingTestApp::addAbsolute6DoFConstraints(
    const size_t add_constraint_at_every_nth_vertex) {
  CHECK_GT(add_constraint_at_every_nth_vertex, 1u);

  vi_map::VIMapManager map_manager;
  vi_map::VIMapManager::MapWriteAccess map =
      map_manager.getMapWriteAccess(map_key_);

  // Fixed covariance.
  Eigen::Matrix<double, 6, 6> T_G_S_covariance;
  T_G_S_covariance.setIdentity();
  T_G_S_covariance *= 1e-2 * 1e-2;

  // Add sensor.
  aslam::SensorId sensor_id;
  aslam::generateId(&sensor_id);
  vi_map::Absolute6DoF::UniquePtr sensor_template =
      aligned_unique<vi_map::Absolute6DoF>(sensor_id, "no_topic");

  vi_map::MissionIdList mission_ids;
  map->getAllMissionIds(&mission_ids);

  for (const vi_map::MissionId& mission_id : mission_ids) {
    vi_map::VIMission& mission = map->getMission(mission_id);
    CHECK(mission.hasImu())
        << "This function assumes the mission baseframe is equal to the IMU "
           "frame, hence it requires an IMU to be present.";
    const aslam::SensorId& base_sensor_id = mission.getImuId();

    aslam::Transformation T_B_S;
    T_B_S.setRandom();

    vi_map::Absolute6DoF::UniquePtr sensor(sensor_template->cloneWithNewIds());
    const aslam::SensorId sensor_id = sensor->getId();
    map->getSensorManager().addSensor<vi_map::Absolute6DoF>(
        std::move(sensor), base_sensor_id, T_B_S);
    mission.setAbsolute6DoFSensor(sensor_id);
  }

  pose_graph::VertexIdList all_vertex_ids;
  map->getAllVertexIds(&all_vertex_ids);

  // Sort by vertex ids to get a somewhat random order of vertices;
  std::sort(all_vertex_ids.begin(), all_vertex_ids.end());

  size_t num_abs_constraints_added = 0u;
  for (size_t vertex_id = 0u; vertex_id < all_vertex_ids.size();
       vertex_id += add_constraint_at_every_nth_vertex) {
    // Get vertex information.
    const pose_graph::VertexId vertex_id_a = all_vertex_ids[vertex_id];
    const aslam::Transformation T_G_B = map->getVertex_T_G_I(vertex_id_a);
    vi_map::Vertex& vertex = map->getVertex(vertex_id_a);

    // Get corresponding absolute pose sensor.
    const vi_map::VIMission& mission = map->getMission(vertex.getMissionId());
    CHECK(mission.hasAbsolute6DoFSensor());
    const aslam::SensorId& sensor_id = mission.getAbsolute6DoFSensor();
    const aslam::Transformation T_B_S =
        map->getSensorManager().getSensor_T_B_S(sensor_id);

    CHECK(vertex_id_a.isValid());
    CHECK(sensor_id.isValid());

    const aslam::Transformation T_G_S = T_G_B * T_B_S;

    const vi_map::Absolute6DoFMeasurement abs_6dof_measurement(
        sensor_id, vertex.getMinTimestampNanoseconds(), T_G_S,
        T_G_S_covariance);
    vertex.addAbsolute6DoFMeasurement(abs_6dof_measurement);

    ++num_abs_constraints_added;
  }

  LOG(INFO) << "Added " << num_abs_constraints_added
            << " absolute 6DoF constraints to pose graph.";
}

void VIMappingTestApp::addLoopClosureEdges(
    const size_t add_lc_edge_between_every_nth_vertex) {
  CHECK_GT(add_lc_edge_between_every_nth_vertex, 1u);

  vi_map::VIMapManager map_manager;
  vi_map::VIMapManager::MapWriteAccess map =
      map_manager.getMapWriteAccess(map_key_);

  const double kSwitchVariable = 1.0;
  const double kSwitchVariableVariance = 1e-3;
  Eigen::Matrix<double, 6, 6> T_B_a_B_b_covariance;
  T_B_a_B_b_covariance.setIdentity();
  T_B_a_B_b_covariance *= 1e-2 * 1e-2;

  pose_graph::VertexIdList all_vertex_ids;
  map->getAllVertexIds(&all_vertex_ids);

  // Sort by vertex ids to get a somewhat random order of vertices;
  std::sort(all_vertex_ids.begin(), all_vertex_ids.end());

  size_t num_lc_edges_added = 0u;
  for (size_t vertex_a_idx = 0u; vertex_a_idx < all_vertex_ids.size();
       vertex_a_idx += add_lc_edge_between_every_nth_vertex) {
    size_t vertex_b_idx = vertex_a_idx + add_lc_edge_between_every_nth_vertex;
    if (vertex_b_idx >= all_vertex_ids.size()) {
      break;
    }

    const pose_graph::VertexId vertex_id_a = all_vertex_ids[vertex_a_idx];
    const pose_graph::VertexId vertex_id_b = all_vertex_ids[vertex_b_idx];
    const aslam::Transformation T_G_B_a = map->getVertex_T_G_I(vertex_id_a);
    const aslam::Transformation T_G_B_b = map->getVertex_T_G_I(vertex_id_b);
    const aslam::Transformation T_B_a_B_b = T_G_B_a.inverse() * T_G_B_b;

    CHECK(vertex_id_a.isValid());
    CHECK(vertex_id_b.isValid());

    pose_graph::EdgeId edge_id;
    aslam::generateId(&edge_id);
    vi_map::LoopClosureEdge::UniquePtr loop_closure_edge(
        new vi_map::LoopClosureEdge(
            edge_id, vertex_id_a, vertex_id_b, kSwitchVariable,
            kSwitchVariableVariance, T_B_a_B_b, T_B_a_B_b_covariance));

    map->addEdge(std::move(loop_closure_edge));

    ++num_lc_edges_added;
  }

  LOG(INFO) << "Added " << num_lc_edges_added << " lc edges to pose graph.";
}

pose_graph::EdgeId VIMappingTestApp::addWrongLoopClosureEdge() {
  vi_map::VIMapManager map_manager;
  vi_map::VIMapManager::MapWriteAccess map =
      map_manager.getMapWriteAccess(map_key_);

  const double kSwitchVariable = 1.0;
  const double kSwitchVariableVariance = 1e-3;
  Eigen::Matrix<double, 6, 6> T_Ia_Ib_covariance;
  T_Ia_Ib_covariance.setIdentity();
  T_Ia_Ib_covariance *= 1e-2 * 1e-2;
  pose::Transformation T_Ia_Ib;

  vi_map::MissionIdList mission_ids;
  map->getAllMissionIds(&mission_ids);

  const vi_map::MissionId& first_mission_id = *mission_ids.begin();

  pose_graph::VertexId current_vertex_id =
      map->getMission(first_mission_id).getRootVertexId();
  pose_graph::VertexIdList sorted_vertex_ids;
  do {
    sorted_vertex_ids.push_back(current_vertex_id);
  } while (map->getNextVertex(current_vertex_id, &current_vertex_id));

  const unsigned int index_offset_vertex_from =
      static_cast<unsigned int>(0.2 * sorted_vertex_ids.size());
  const unsigned int index_offset_vertex_to =
      static_cast<unsigned int>(0.9 * sorted_vertex_ids.size());

  CHECK_LT(index_offset_vertex_from, sorted_vertex_ids.size());
  CHECK_LT(index_offset_vertex_to, sorted_vertex_ids.size());

  const pose_graph::VertexId vertex_id_from =
      sorted_vertex_ids[index_offset_vertex_from];
  const pose_graph::VertexId vertex_id_to =
      sorted_vertex_ids[index_offset_vertex_to];

  CHECK(vertex_id_from.isValid());
  CHECK(vertex_id_to.isValid());

  pose_graph::EdgeId edge_id;
  aslam::generateId(&edge_id);
  vi_map::LoopClosureEdge::UniquePtr loop_closure_edge(
      new vi_map::LoopClosureEdge(
          edge_id, vertex_id_from, vertex_id_to, kSwitchVariable,
          kSwitchVariableVariance, T_Ia_Ib, T_Ia_Ib_covariance));

  map->addEdge(std::move(loop_closure_edge));

  return edge_id;
}

const Eigen::Vector3d& VIMappingTestApp::getLandmarkReferencePosition(
    const vi_map::LandmarkId& landmark_id) const {
  CHECK(landmark_id.isValid());
  LandmarkIdPositionPairs::const_iterator it;
  it = landmark_reference_positions_.find(landmark_id);
  CHECK(it != landmark_reference_positions_.end());

  return it->second;
}

const pose::Transformation& VIMappingTestApp::getVertexReferencePose(
    const pose_graph::VertexId& vertex_id) const {
  CHECK(vertex_id.isValid());

  VertexIdPosePairs::const_iterator it;
  it = vertex_reference_poses_.find(vertex_id);
  CHECK(it != vertex_reference_poses_.end());

  return it->second;
}

void VIMappingTestApp::testIfKeyframesMatchReference(double precision_m) const {
  // Store the reference keyframe poses and landmark positions.
  const vi_map::VIMapManager map_manager;
  vi_map::VIMapManager::MapReadAccess map =
      map_manager.getMapReadAccess(map_key_);

  double max_error = 0.0;
  double error_sum = 0.0;

  pose_graph::VertexIdList vertex_ids;
  map->getAllVertexIds(&vertex_ids);
  for (const pose_graph::VertexId& vertex_id : vertex_ids) {
    const pose::Transformation& T_M_I = map->getVertex(vertex_id).get_T_M_I();
    EXPECT_NEAR_ASLAM_TRANSFORMATION(
        getVertexReferencePose(vertex_id), T_M_I, precision_m);

    const double error =
        (((getVertexReferencePose(vertex_id)).getTransformationMatrix()) -
         ((T_M_I).getTransformationMatrix()))
            .cwiseAbs()
            .maxCoeff();

    max_error = std::max(max_error, error);
    error_sum += error;
  }

  LOG(INFO) << "Max error of keyframes: " << max_error;
  LOG(INFO) << "Mean error of keyframes: "
            << error_sum / static_cast<double>(vertex_ids.size());
}

void VIMappingTestApp::testIfLandmarksMatchReference(
    double max_distance, double min_required_fraction) const {
  CHECK_GT(min_required_fraction, 0.0 - std::numeric_limits<double>::epsilon());
  CHECK_LE(min_required_fraction, 1.0 + std::numeric_limits<double>::epsilon());
  // Store the reference keyframe poses and landmark positions.
  const vi_map::VIMapManager map_manager;
  vi_map::VIMapManager::MapReadAccess map =
      map_manager.getMapReadAccess(map_key_);

  double max_error = 0.0;
  double error_sum = 0.0;

  vi_map::LandmarkIdSet landmark_ids;
  map->getAllLandmarkIds(&landmark_ids);
  unsigned int num_failing = 0;
  for (const vi_map::LandmarkId& landmark_id : landmark_ids) {
    const Eigen::Vector3d& p_B_fi = map->getLandmark(landmark_id).get_p_B();
    // Scale precision (see documentation of isApprox in Eigen) to check
    // absolute distance in meters between the expected and actual positions.
    const double scaled_precision =
        max_distance /
        std::min(
            p_B_fi.norm(), getLandmarkReferencePosition(landmark_id).norm());
    if (fabs(min_required_fraction - 1.0) <
        std::numeric_limits<double>::epsilon()) {
      EXPECT_NEAR_EIGEN(
          getLandmarkReferencePosition(landmark_id), p_B_fi, scaled_precision);

    } else {
      if (!getLandmarkReferencePosition(landmark_id)
               .isApprox(p_B_fi, scaled_precision)) {
        ++num_failing;
      }
    }
    const double error = (getLandmarkReferencePosition(landmark_id) - p_B_fi)
                             .cwiseAbs()
                             .maxCoeff();
    max_error = std::max(max_error, error);
    error_sum += error;
  }

  if (!landmark_ids.empty()) {
    EXPECT_LE(
        (static_cast<double>(num_failing) / landmark_ids.size()),
        min_required_fraction);
  }

  LOG(INFO) << "Max error of landmarks positions: " << max_error;
  LOG(INFO) << "Mean error of landmarks positions: "
            << error_sum / static_cast<double>(landmark_ids.size());
}

void VIMappingTestApp::testIfSwitchVariablesLargerThan(
    double min_value, double min_required_fraction) const {
  CHECK_GT(min_required_fraction, 0.0 - std::numeric_limits<double>::epsilon());
  CHECK_LE(min_required_fraction, 1.0 + std::numeric_limits<double>::epsilon());
  CHECK_GT(min_value, 0.0 - std::numeric_limits<double>::epsilon());
  CHECK_LE(min_value, 1.0 + std::numeric_limits<double>::epsilon());

  const vi_map::VIMapManager map_manager;
  vi_map::VIMapManager::MapReadAccess map =
      map_manager.getMapReadAccess(map_key_);
  pose_graph::EdgeIdList all_edge_ids;
  map->getAllEdgeIds(&all_edge_ids);
  unsigned int num_lc_edges = 0;
  unsigned int num_lc_edges_over_threshold = 0;
  for (const pose_graph::EdgeId& edge_id : all_edge_ids) {
    if (map->getEdgeType(edge_id) == pose_graph::Edge::EdgeType::kLoopClosure) {
      ++num_lc_edges;
      const vi_map::LoopClosureEdge& lc_edge =
          map->getEdgeAs<vi_map::LoopClosureEdge>(edge_id);
      if (lc_edge.getSwitchVariable() > min_value) {
        ++num_lc_edges_over_threshold;
      }
    }
  }

  if (num_lc_edges > 0) {
    CHECK_GE(
        (static_cast<double>(num_lc_edges_over_threshold) / num_lc_edges),
        min_required_fraction);
  }
}

double VIMappingTestApp::getSpecificSwitchVariable(
    const pose_graph::EdgeId& edge_id) const {
  const vi_map::VIMapManager map_manager;
  vi_map::VIMapManager::MapReadAccess map =
      map_manager.getMapReadAccess(map_key_);

  CHECK_EQ(
      static_cast<int>(map->getEdgeType(edge_id)),
      static_cast<int>(pose_graph::Edge::EdgeType::kLoopClosure));

  const vi_map::LoopClosureEdge& lc_edge =
      map->getEdgeAs<vi_map::LoopClosureEdge>(edge_id);
  return lc_edge.getSwitchVariable();
}

}  // namespace visual_inertial_mapping
