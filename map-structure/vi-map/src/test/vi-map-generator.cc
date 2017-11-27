#include "vi-map/test/vi-map-generator.h"

#include <aslam/cameras/camera-pinhole.h>
#include <aslam/cameras/distortion-fisheye.h>
#include <aslam/cameras/ncamera.h>
#include <aslam/common/memory.h>
#include <aslam/common/pose-types.h>
#include <maplab-common/accessors.h>

namespace vi_map {

VIMapGenerator::VIMapGenerator(VIMap& map, int seed)  // NOLINT
    : map_(map),
      rng_(seed) {
  Eigen::VectorXd mock_intrinsics(4);
  mock_intrinsics << kMockF, kMockF, kMockC, kMockC;
  Eigen::VectorXd distortion_parameters(1);
  distortion_parameters(0) = 1;
  aslam::Distortion::UniquePtr distortion(
      new aslam::FisheyeDistortion(distortion_parameters));
  aslam::Camera::Ptr camera(
      new aslam::PinholeCamera(
          mock_intrinsics, kCameraWidth, kCameraHeight, distortion));
  aslam::CameraId id;
  id.randomize();
  camera->setId(id);

  std::vector<aslam::Camera::Ptr> cameras;
  Aligned<std::vector, aslam::Transformation> T_C_B_vector;

  cameras.push_back(camera);
  T_C_B_vector.push_back(aslam::Transformation());

  aslam::NCameraId rig_id;
  rig_id.randomize();
  std::string label("Test camera rig");
  n_camera_ = aslam::NCamera::Ptr(
      new aslam::NCamera(rig_id, T_C_B_vector, cameras, label));

  default_edge_T_covariance_p_q_.setZero();
}

MissionId VIMapGenerator::createMission() {
  return createMission(pose::Transformation());
}
MissionId VIMapGenerator::createMission(const pose::Transformation& T_G_M) {
  MissionId id;
  generateId(&id);
  CHECK(missions_.insert(std::make_pair(id, T_G_M)).second);
  return id;
}

pose_graph::VertexId VIMapGenerator::createVertex(
    const MissionId& mission, const pose::Transformation& T_G_I) {
  constexpr int64_t kTimestampNanoseconds = 0;
  return createVertex(mission, T_G_I, kTimestampNanoseconds);
}

pose_graph::VertexId VIMapGenerator::createVertex(
    const MissionId& mission, const pose::Transformation& T_G_I,
    const int64_t timestamp_nanoseconds) {
  CHECK_NE(0u, missions_.count(mission));
  pose_graph::VertexId id;
  generateId(&id);
  CHECK(vertices_.emplace(id, VertexInfo(mission, T_G_I, timestamp_nanoseconds))
            .second);
  if (last_vertex_id_.count(mission) != 0) {
    edges_.push_back(EdgeInfo(mission, last_vertex_id_[mission], id));
  }
  last_vertex_id_[mission] = id;
  VLOG(10) << "Added vertex " << id << ".";
  return id;
}

vi_map::LandmarkId VIMapGenerator::createLandmark(
    const Eigen::Vector3d& p_G_fi, const pose_graph::VertexId& storing_vertex,
    const std::initializer_list<pose_graph::VertexId>& non_storing_observers) {
  return createLandmark(
      p_G_fi, storing_vertex,
      static_cast<const pose_graph::VertexIdList&>(non_storing_observers));
}

vi_map::LandmarkId VIMapGenerator::createLandmark(
    const Eigen::Vector3d& p_G_fi, const pose_graph::VertexId& storing_vertex,
    const pose_graph::VertexIdList& non_storing_observers) {
  return createLandmarkWithMissingReferences(
      p_G_fi, storing_vertex, non_storing_observers,
      pose_graph::VertexIdList());
}

vi_map::LandmarkId VIMapGenerator::createLandmarkWithMissingReferences(
    const Eigen::Vector3d& p_G_fi, const pose_graph::VertexId& storing_vertex,
    const std::initializer_list<pose_graph::VertexId>& non_storing_observers,
    const std::initializer_list<pose_graph::VertexId>&
        non_referring_observers) {
  return createLandmarkWithMissingReferences(
      p_G_fi, storing_vertex,
      static_cast<const pose_graph::VertexIdList&>(non_storing_observers),
      static_cast<const pose_graph::VertexIdList&>(non_referring_observers));
}

vi_map::LandmarkId VIMapGenerator::createLandmarkWithMissingReferences(
    const Eigen::Vector3d& p_G_fi, const pose_graph::VertexId& storing_vertex,
    const pose_graph::VertexIdList& non_storing_observers,
    const pose_graph::VertexIdList& non_referring_observers) {
  std::vector<VertexInfoMap::iterator> observer_its;
  for (const pose_graph::VertexId& observer : non_storing_observers) {
    CHECK_NE(storing_vertex, observer)
        << "The storing vertex id " << storing_vertex.hexString()
        << " found among observing vertices. "
        << "The storing vertex is automatically assumed to be an observer "
        << "and shouldn't be explicitly added to the observer vertex ids.";
    observer_its.push_back(vertices_.find(observer));
    CHECK(observer_its.back() != vertices_.end());
  }
  for (const pose_graph::VertexId& observer : non_referring_observers) {
    CHECK_NE(storing_vertex, observer);
    observer_its.push_back(vertices_.find(observer));
    CHECK(observer_its.back() != vertices_.end());
  }
  vi_map::LandmarkId landmark_id;
  generateId(&landmark_id);
  VIMap::DescriptorType descriptor(kDescriptorSize, 1);
  descriptor.setRandom();
  CHECK(
      landmarks_
          .emplace(
              landmark_id, LandmarkInfo(
                               p_G_fi, descriptor, storing_vertex,
                               non_storing_observers, non_referring_observers))
          .second);

  if (storing_vertex.isValid()) {
    VertexInfoMap::iterator storing_vertex_it = vertices_.find(storing_vertex);
    CHECK(storing_vertex_it != vertices_.end());
    CHECK(storing_vertex_it->second.landmarks.emplace(landmark_id).second);
  } else {
    CHECK(non_storing_observers.empty()) << "Landmarks that are not stored "
                                            "cannot be referred to.";
  }
  for (VertexInfoMap::iterator it : observer_its) {
    CHECK(it->second.landmarks.emplace(landmark_id).second);
  }
  return landmark_id;
}

vi_map::LandmarkId VIMapGenerator::createLandmarkWithoutReferences(
    const Eigen::Vector3d& p_G_fi,
    const pose_graph::VertexIdList& non_referring_observers) {
  return createLandmarkWithMissingReferences(
      p_G_fi, pose_graph::VertexId(), pose_graph::VertexIdList(),
      static_cast<const pose_graph::VertexIdList&>(non_referring_observers));
}

void VIMapGenerator::setCameraRig(
    const std::shared_ptr<aslam::NCamera>& n_camera) {
  CHECK(n_camera);
  n_camera_ = n_camera;
}

void VIMapGenerator::setDefaultTransformationCovariancePQ(
    const Eigen::Matrix<double, 6, 6>& covariance_p_q) {
  default_edge_T_covariance_p_q_ = covariance_p_q;
}

template <>
TransformationEdge* VIMapGenerator::generateEdge<TransformationEdge>(
    const EdgeInfo& edge_info) const {
  pose_graph::EdgeId id;
  generateId(&id);
  pose::Transformation T_A_B =
      common::getChecked(vertices_, edge_info.from).T_G_I.inverse() *
      common::getChecked(vertices_, edge_info.to).T_G_I;
  return new TransformationEdge(
      vi_map::Edge::EdgeType::kOdometry, id, edge_info.from, edge_info.to,
      T_A_B, default_edge_T_covariance_p_q_);
}

template <>
ViwlsEdge* VIMapGenerator::generateEdge<ViwlsEdge>(
    const EdgeInfo& edge_info) const {
  pose_graph::EdgeId id;
  generateId(&id);
  return new ViwlsEdge(id, edge_info.from, edge_info.to);
}

template <>
Mission::BackBone VIMapGenerator::backBoneType<TransformationEdge>() {
  return Mission::BackBone::kOdometry;
}

template <>
Mission::BackBone VIMapGenerator::backBoneType<ViwlsEdge>() {
  return Mission::BackBone::kViwls;
}

template <typename EdgeType>
void VIMapGenerator::generateMap() const {
  CHECK_EQ(0u, map_.numVertices());
  CHECK_EQ(0u, map_.numEdges());
  CHECK_EQ(0u, map_.numLandmarks());
  CHECK_EQ(0u, map_.numMissions());

  SensorManager& sensor_manager = map_.getSensorManager();

  ImuSigmas imu_sigmas;
  imu_sigmas.gyro_noise_density = 0.1;
  imu_sigmas.gyro_bias_random_walk_noise_density = 0.1;
  imu_sigmas.acc_noise_density = 0.1;
  imu_sigmas.acc_bias_random_walk_noise_density = 0.1;
  constexpr char kImuHardwareId[] = "imu0";
  SensorId imu_sensor_id;
  common::generateId(&imu_sensor_id);
  Imu::UniquePtr imu_sensor = aligned_unique<Imu>(
      imu_sensor_id, static_cast<std::string>(kImuHardwareId));
  imu_sensor->setImuSigmas(imu_sigmas);
  CHECK(imu_sensor_id.isValid());
  sensor_manager.addSensor(std::move(imu_sensor));

  // MISSIONS
  CHECK(n_camera_);
  for (const MissionInfoMap::value_type& mission_pair : missions_) {
    map_.addNewMissionWithBaseframe(
        mission_pair.first, mission_pair.second,
        Eigen::Matrix<double, 6, 6>::Zero(), n_camera_,
        backBoneType<EdgeType>());
    sensor_manager.associateExistingSensorWithMission(
        imu_sensor_id, mission_pair.first);
  }

  // VERTICES
  typedef std::unordered_map<pose_graph::VertexId, vi_map::Vertex::UniquePtr>
      GeneratedVertexMap;
  GeneratedVertexMap generated_vertices;
  ObservationIndexMap observation_index;
  for (const VertexInfoMap::value_type& vertex_pair : vertices_) {
    const pose_graph::VertexId& id = vertex_pair.first;
    const VertexInfo& info = vertex_pair.second;
    const MissionId& mission_id = info.mission;

    vi_map::Vertex::UniquePtr& vertex_ptr = generated_vertices[id];
    Eigen::Matrix2Xd image_points;
    aslam::VisualFrame::DescriptorsT descriptors;
    generateLandmarkObservations(
        id, &image_points, &descriptors, &observation_index);
    LandmarkIdList observed_landmark_ids(image_points.cols());
    aslam::FrameId frame_id;
    common::generateId(&frame_id);
    vertex_ptr.reset(
        new Vertex(
            id, Eigen::Matrix<double, 6, 1>::Zero(), image_points,
            Eigen::VectorXd::Zero(observed_landmark_ids.size()), descriptors,
            observed_landmark_ids, vertex_pair.second.mission, frame_id,
            info.timestamp_nanoseconds, n_camera_));

    const MissionBaseFrame& base_frame =
        map_.getMissionBaseFrame(map_.getMission(mission_id).getBaseFrameId());
    pose::Transformation T_M_I = base_frame.get_T_G_M().inverse() * info.T_G_I;
    vertex_ptr->set_T_M_I(T_M_I);

    map_.addVertex(std::move(vertex_ptr));

    CHECK(map_.getVertex(id).getNCameras());
  }

  // EDGES
  for (const EdgeInfo& edge : edges_) {
    vi_map::Edge* edge_ptr(generateEdge<EdgeType>(edge));
    map_.addEdge(vi_map::Edge::UniquePtr(edge_ptr));
  }

  // ROOT_VERTICES
  MissionIdList all_mission_ids;
  map_.getAllMissionIds(&all_mission_ids);
  pose_graph::VertexIdList all_vertex_ids;
  map_.getAllVertexIds(&all_vertex_ids);

  for (const MissionId& mission_id : all_mission_ids) {
    pose_graph::VertexIdList mission_vertex_ids;
    for (const pose_graph::VertexId& vertex_id : all_vertex_ids) {
      if (map_.getVertex(vertex_id).getMissionId() == mission_id) {
        mission_vertex_ids.push_back(vertex_id);
      }
    }
    for (const pose_graph::VertexId& vertex_id : mission_vertex_ids) {
      pose_graph::EdgeIdSet incoming_edge_ids;
      map_.getVertex(vertex_id).getIncomingEdges(&incoming_edge_ids);
      if (incoming_edge_ids.empty()) {
        map_.getMission(mission_id).setRootVertexId(vertex_id);
        break;
      }
    }
  }

  // LANDMARKS
  for (const LandmarkInfoMap::value_type& landmark_pair : landmarks_) {
    const vi_map::LandmarkId landmark_id = landmark_pair.first;
    const LandmarkInfo landmark_info = landmark_pair.second;
    if (!landmark_info.storing_vertex_id.isValid()) {
      continue;
    }

    Landmark landmark;
    landmark.setId(landmark_id);
    VertexInfoMap::const_iterator vertex_info_it =
        vertices_.find(landmark_info.storing_vertex_id);
    CHECK(vertex_info_it != vertices_.end());
    pose::Transformation T_G_I(vertex_info_it->second.T_G_I);
    landmark.set_p_B(T_G_I.inverse() * landmark_info.p_G_fi);

    const ObservationIndices& storing_vertex_indices =
        observation_index[landmark_info.storing_vertex_id][landmark_id];

    map_.addNewLandmark(
        landmark, landmark_info.storing_vertex_id,
        storing_vertex_indices.frame_id, storing_vertex_indices.keypoint_id);

    for (const pose_graph::VertexId vertex_id :
         landmark_info.non_storing_vertex_ids) {
      const ObservationIndices& observer_vertex_indices =
          observation_index[vertex_id][landmark_id];
      map_.associateKeypointWithExistingLandmark(
          vertex_id, observer_vertex_indices.frame_id,
          observer_vertex_indices.keypoint_id, landmark_id);
    }
  }
}

template void VIMapGenerator::generateMap<TransformationEdge>() const;
template void VIMapGenerator::generateMap<ViwlsEdge>() const;

Eigen::Matrix2Xd VIMapGenerator::centerMeasurementsOnPrincipalPoint(
    const Eigen::Matrix2Xd& image_points, const size_t frame_index) const {
  const aslam::PinholeCamera* camera =
      dynamic_cast<const aslam::PinholeCamera*>(  // NOLINT
          &n_camera_->getCamera(frame_index));
  CHECK(camera != nullptr) << "Only Pinhole camera currently supported!";
  return image_points.colwise() - Eigen::Vector2d(camera->cu(), camera->cv());
}

void VIMapGenerator::generateLandmarkObservations(
    const pose_graph::VertexId& vertex_id, Eigen::Matrix2Xd* image_points,
    aslam::VisualFrame::DescriptorsT* descriptors,
    ObservationIndexMap* observation_index) const {
  CHECK_NOTNULL(image_points);
  CHECK_NOTNULL(descriptors);
  CHECK_NOTNULL(observation_index);

  VertexInfoMap::const_iterator vertex_info_it = vertices_.find(vertex_id);
  CHECK(vertex_info_it != vertices_.end());
  const VertexInfo& vertex_info = vertex_info_it->second;
  CHECK_EQ(n_camera_->numCameras(), 1u)
      << "Only one camera currently supported!";
  const pose::Transformation T_C_G(
      n_camera_->get_T_C_B(0) * vertex_info.T_G_I.inverse());
  size_t total = vertex_info.landmarks.size();
  image_points->resize(2, total);
  descriptors->resize(kDescriptorSize, total);
  VLOG(10) << "Vertex " << vertex_id << " gets " << total << " descriptors.";

  unsigned int i = 0;
  for (const vi_map::LandmarkId& landmark_id : vertex_info.landmarks) {
    LandmarkInfoMap::const_iterator it = landmarks_.find(landmark_id);
    CHECK(it != landmarks_.cend());
    const LandmarkInfo& info = it->second;
    projectLandmark(info, T_C_G, image_points, i);
    descriptors->block<kDescriptorSize, 1>(0, i) =
        Eigen::Map<const Eigen::Matrix<unsigned char, kDescriptorSize, 1>>(
            info.descriptor.data());
    (*observation_index)[vertex_id][landmark_id] = {0, i};
    ++i;
  }
}

void VIMapGenerator::projectLandmark(
    const LandmarkInfo& landmark_info, const pose::Transformation& T_C_G,
    Eigen::Matrix2Xd* keypoints, size_t index) const {
  Eigen::Vector2d image_point_vec;
  CHECK_EQ(n_camera_->numCameras(), 1u)
      << "Only one camera currently supported!";
  CHECK(
      n_camera_->getCamera(0)
          .project3(T_C_G.transform(landmark_info.p_G_fi), &image_point_vec)
          .isKeypointVisible());
  keypoints->block<2, 1>(0, index) = image_point_vec;
}

VIMapGenerator::VertexInfo::VertexInfo(
    const MissionId& _mission, const pose::Transformation& _T_G_I,
    const int64_t _timestamp_nanoseconds)
    : mission(_mission),
      T_G_I(_T_G_I),
      timestamp_nanoseconds(_timestamp_nanoseconds) {}

VIMapGenerator::LandmarkInfo::LandmarkInfo(
    const Eigen::Vector3d& _p_G_fi, const VIMap::DescriptorType& _descriptor,
    const pose_graph::VertexId& _storing_vertex_id,
    const pose_graph::VertexIdList& _non_storing_vertex_ids,
    const pose_graph::VertexIdList& _non_referring_vertex_ids)
    : p_G_fi(_p_G_fi),
      descriptor(_descriptor),
      storing_vertex_id(_storing_vertex_id),
      non_storing_vertex_ids(_non_storing_vertex_ids),
      non_referring_vertex_ids(_non_referring_vertex_ids) {}

}  // namespace vi_map
