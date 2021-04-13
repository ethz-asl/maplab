#include <memory>
#include <mutex>
#include <random>
#include <unordered_map>
#include <vector>

#include <Eigen/Core>

#include <aslam/cameras/camera-pinhole.h>
#include <aslam/cameras/distortion-fisheye.h>
#include <aslam/frames/visual-frame.h>
#include <vi-map/landmark-index.h>
#include <vi-map/landmark.h>
#include <vi-map/mission-baseframe.h>
#include <vi-map/mission.h>
#include <vi-map/unique-id.h>
#include <vi-map/vertex.h>
#include <vi-map/viwls-edge.h>

#include <loop-closure-handler/loop-closure-constraint.h>
#include <loop-closure-handler/loop-closure-handler.h>
#include <maplab-common/pose_types.h>
#include <maplab-common/quaternion-math.h>
#include <maplab-common/test/testing-entrypoint.h>
#include <maplab-common/test/testing-predicates.h>

DECLARE_double(lc_min_image_time_seconds);
DECLARE_double(vi_map_landmark_quality_max_distance_from_closest_observer);
DECLARE_bool(lc_filter_underconstrained_landmarks);

struct ExpectedLandmarkMergeTriple {
  pose_graph::VertexId vertex_id;
  int idx;
  vi_map::LandmarkId new_landmark_id;
};

namespace vi_map {
void addObservedLandmarkId(
    const LandmarkId& landmark_id, vi_map::Vertex* vertex_ptr) {
  CHECK_NOTNULL(vertex_ptr);
  static constexpr unsigned int kVisualFrameIndex = 0;
  vertex_ptr->addObservedLandmarkId(kVisualFrameIndex, landmark_id);
}
}  // namespace vi_map

static constexpr unsigned int kNumOfLandmarks = 200;
static constexpr unsigned int kNumOfDuplicateLandmarks = 50;
static constexpr unsigned int kNumOfMapVertices = 5;
static constexpr unsigned int kNumOfQueryVertices = 1;

static constexpr int kVisualFrameIndex = 0;

class LoopClosureHandlerTest : public ::testing::Test {
 protected:
  typedef aslam::FisheyeDistortion DistortionType;
  typedef aslam::PinholeCamera CameraType;
  typedef std::unordered_map<vi_map::LandmarkId, vi_map::LandmarkId>
      LandmarkToLandmarkMap;

  LoopClosureHandlerTest()
      : map_(),
        posegraph_(map_.posegraph),
        landmark_index_(map_.landmark_index),
        missions_(map_.missions),
        mission_base_frames_(map_.mission_base_frames),
        submap_(),
        submap_posegraph_(submap_.posegraph),
        submap_landmark_index_(submap_.landmark_index),
        submap_missions_(submap_.missions),
        submap_mission_base_frames_(submap_.mission_base_frames),
        gen_(1),
        dis_(-1.0, 1.0) {
    FLAGS_lc_min_image_time_seconds = 0.0;
    FLAGS_vi_map_landmark_quality_max_distance_from_closest_observer = 20.0;
    FLAGS_lc_filter_underconstrained_landmarks = false;
  }

  virtual void SetUp() {
    constructCamera();
    createMission();
    populatePosegraph();
    generateAndProjectLandmarksToMapKeyframes();
    addDuplicateLandmarksToQueryKeyframes();
    handler_ = std::shared_ptr<loop_closure_handler::LoopClosureHandler>(
        new loop_closure_handler::LoopClosureHandler(
            &map_, &landmark_id_old_to_new_));
    CHECK_LE(kNumOfDuplicateLandmarks, kNumOfLandmarks);
  }

  bool hasLandmark(const vi_map::LandmarkId& landmark_id) const;

  pose_graph::VertexId getVertexIdForLandmark(
      const vi_map::LandmarkId& landmark_id) const;

  void constructCamera();
  void createMission();
  void populatePosegraph();
  void populateSubmap();
  void generateAndProjectLandmarksToMapKeyframes();
  unsigned int addKeypointToVertex(
      const Eigen::Vector3d& G_p_fi, const vi_map::LandmarkId& landmark_id,
      vi_map::Vertex* vertex_ptr);
  void addLandmarkToVertex(
      const Eigen::Vector3d& G_p_fi, const vi_map::LandmarkId& landmark_id,
      vi_map::Vertex* vertex_ptr);
  unsigned int addKeypointToSubmapVertex(
      const Eigen::Vector3d& G_p_fi, const vi_map::LandmarkId& landmark_id,
      vi_map::Vertex* vertex_ptr);
  void addLandmarkToSubmapVertex(
      const Eigen::Vector3d& G_p_fi, const vi_map::LandmarkId& landmark_id,
      vi_map::Vertex* vertex_ptr);
  void addDuplicateLandmarksToQueryKeyframes();

  bool testLandmarkConsistency();
  vi_map::VIMap map_;
  vi_map::PoseGraph& posegraph_;
  vi_map::LandmarkIndex& landmark_index_;
  vi_map::VIMissionMap& missions_;
  vi_map::MissionBaseFrameMap& mission_base_frames_;

  vi_map::VIMap submap_;
  vi_map::PoseGraph& submap_posegraph_;
  vi_map::LandmarkIndex& submap_landmark_index_;
  vi_map::VIMissionMap& submap_missions_;
  vi_map::MissionBaseFrameMap& submap_mission_base_frames_;

  std::shared_ptr<loop_closure_handler::LoopClosureHandler> handler_;
  LandmarkToLandmarkMap landmark_id_old_to_new_;

  std::vector<pose_graph::VertexId> vertex_ids_;
  std::vector<pose_graph::VertexId> submap_vertex_ids_;
  vi_map::MissionId mission_id_;
  aslam::NCamera::Ptr cameras_;
  aslam::SensorId n_camera_id_;
  vi_map::LoopClosureConstraintVector constraints_;
  LandmarkToLandmarkMap duplicate_landmark_to_landmark_map_;
  std::vector<ExpectedLandmarkMergeTriple> expected_landmark_merges_;
  std::mt19937 gen_;
  std::uniform_real_distribution<> dis_;

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

bool LoopClosureHandlerTest::hasLandmark(
    const vi_map::LandmarkId& landmark_id) const {
  return landmark_index_.hasLandmark(landmark_id);
}

pose_graph::VertexId LoopClosureHandlerTest::getVertexIdForLandmark(
    const vi_map::LandmarkId& landmark_id) const {
  return landmark_index_.getStoringVertexId(landmark_id);
}

void LoopClosureHandlerTest::constructCamera() {
  double distortion_param = 0.94;
  double fu = 50;
  double fv = 50;
  int res_u = 1000;
  int res_v = 1000;
  double cu = res_u / 2.0;
  double cv = res_v / 2.0;

  Eigen::VectorXd distortion_parameters(1);
  distortion_parameters << distortion_param;

  aslam::Distortion::UniquePtr distortion(
      new DistortionType(distortion_parameters));

  Eigen::VectorXd intrinsics(4);
  intrinsics << fu, fv, cu, cv;

  aslam::Camera::Ptr camera = std::shared_ptr<CameraType>(
      new CameraType(intrinsics, res_u, res_v, distortion));

  aslam::CameraId camera_id;
  aslam::generateId(&camera_id);
  camera->setId(camera_id);

  std::vector<aslam::Camera::Ptr> camera_vector;
  camera_vector.push_back(camera);
  aslam::TransformationVector T_C_B_vector;
  aslam::Transformation T_C_B(
      Eigen::Quaterniond(sqrt(2) / 2, 0, sqrt(2) / 2, 0),
      Eigen::Vector3d(1, 2, 3));
  T_C_B_vector.push_back(T_C_B);
  aslam::generateId(&n_camera_id_);
  cameras_.reset(new aslam::NCamera(
      n_camera_id_, T_C_B_vector, camera_vector, "Test camera rig"));
  map_.getSensorManager().addSensor<aslam::NCamera>(
      aligned_unique<aslam::NCamera>(*cameras_), n_camera_id_, T_C_B);
  submap_.getSensorManager().addSensor<aslam::NCamera>(
      aligned_unique<aslam::NCamera>(*cameras_), n_camera_id_, T_C_B);
}

void LoopClosureHandlerTest::createMission() {
  vi_map::MissionBaseFrame baseframe;
  vi_map::MissionBaseFrameId baseframe_id;
  aslam::generateId(&baseframe_id);
  baseframe.setId(baseframe_id);

  baseframe.set_p_G_M(Eigen::Matrix<double, 3, 1>::Zero());
  baseframe.set_q_G_M(Eigen::Quaterniond::Identity());

  vi_map::VIMission::UniquePtr mission_ptr(new vi_map::VIMission);

  aslam::generateId(&mission_id_);
  mission_ptr->setId(mission_id_);
  mission_ptr->setNCameraId(n_camera_id_);
  mission_ptr->setBaseFrameId(baseframe_id);

  missions_.emplace(mission_ptr->id(), std::move(mission_ptr));
  mission_base_frames_.emplace(baseframe.id(), baseframe);
}

void LoopClosureHandlerTest::populatePosegraph() {
  Eigen::Quaterniond orientation(1, 0, 0, 0);
  Eigen::Vector3d position(0, 0, 0);

  pose_graph::VertexId vertex_id;
  vi_map::Vertex::UniquePtr vertex(new vi_map::Vertex(cameras_));

  aslam::generateId(&vertex_id);
  vertex->setId(vertex_id);
  vertex->set_p_M_I(position);
  vertex->set_q_M_I(orientation);
  vertex->setMissionId(mission_id_);
  posegraph_.addVertex(std::move(vertex));

  vertex_ids_.resize(kNumOfMapVertices + kNumOfQueryVertices);
  vertex_ids_[0] = vertex_id;
  map_.getMission(mission_id_).setRootVertexId(vertex_ids_[0]);
  for (unsigned int i = 1; i < kNumOfMapVertices + kNumOfQueryVertices; ++i) {
    vi_map::Vertex::UniquePtr vertex(new vi_map::Vertex(cameras_));
    aslam::generateId(&vertex_id);
    vertex->setId(vertex_id);
    position.x() += i * 0.2;
    vertex->set_p_M_I(position);
    vertex->set_q_M_I(orientation);
    vertex->setMissionId(mission_id_);
    posegraph_.addVertex(std::move(vertex));
    vertex_ids_[i] = vertex_id;

    pose_graph::EdgeId edge_id;
    aslam::generateId(&edge_id);

    Eigen::Matrix<int64_t, 1, Eigen::Dynamic> imu_timestamps;
    Eigen::Matrix<double, 6, Eigen::Dynamic> imu_data;
    vi_map::MissionId mission_id;
    aslam::generateId(&mission_id);

    vi_map::ViwlsEdge::UniquePtr edge(new vi_map::ViwlsEdge(
        edge_id, vertex_ids_[i - 1], vertex_ids_[i], imu_timestamps, imu_data));
    posegraph_.addEdge(std::move(edge));
  }
}

void LoopClosureHandlerTest::populateSubmap() {
  vi_map::MissionBaseFrame baseframe = mission_base_frames_.begin()->second;

  vi_map::VIMission::UniquePtr mission_ptr(new vi_map::VIMission);
  *mission_ptr = *missions_.begin()->second;
  submap_missions_.emplace(mission_ptr->id(), std::move(mission_ptr));
  submap_mission_base_frames_.emplace(baseframe.id(), baseframe);

  std::unordered_map<vi_map::LandmarkId, vi_map::LandmarkId>
      base_landmark_to_submap_id_map;
  submap_vertex_ids_.resize(kNumOfMapVertices + kNumOfQueryVertices);
  for (unsigned int i = 0; i < kNumOfMapVertices + kNumOfQueryVertices; ++i) {
    vi_map::Vertex::UniquePtr vertex(new vi_map::Vertex(cameras_));

    pose_graph::VertexId vertex_id;
    if (i == 0) {
      vertex_id = vertex_ids_[kNumOfMapVertices + kNumOfQueryVertices - 1];
    } else {
      aslam::generateId(&vertex_id);
    }
    vertex->setId(vertex_id);
    vi_map::Vertex& base_vertex = map_.getVertex(
        vertex_ids_[kNumOfMapVertices + kNumOfQueryVertices - 1 - i]);
    vertex->set_p_M_I(base_vertex.get_p_M_I());
    vertex->set_q_M_I(base_vertex.get_q_M_I());
    vertex->setMissionId(mission_id_);
    submap_posegraph_.addVertex(std::move(vertex));
    submap_vertex_ids_[i] = vertex_id;
    if (i == 0)
      submap_.getMission(mission_id_).setRootVertexId(submap_vertex_ids_[0]);
    vi_map::LandmarkIdList owned_landmark_ids;
    base_vertex.getStoredLandmarkIdList(&owned_landmark_ids);
    for (vi_map::LandmarkId& landmark_id : owned_landmark_ids) {
      vi_map::LandmarkId new_landmark_id;
      aslam::generateId(&new_landmark_id);
      base_landmark_to_submap_id_map[landmark_id] = new_landmark_id;
      addLandmarkToSubmapVertex(
          base_vertex.getLandmark_p_LM_fi(landmark_id), new_landmark_id,
          &submap_.getVertex(submap_vertex_ids_[i]));
    }
    if (i > 0) {
      pose_graph::EdgeId edge_id;
      aslam::generateId(&edge_id);

      Eigen::Matrix<int64_t, 1, Eigen::Dynamic> imu_timestamps;
      Eigen::Matrix<double, 6, Eigen::Dynamic> imu_data;
      vi_map::ViwlsEdge::UniquePtr edge(new vi_map::ViwlsEdge(
          edge_id, submap_vertex_ids_[i - 1], submap_vertex_ids_[i],
          imu_timestamps, imu_data));
      submap_posegraph_.addEdge(std::move(edge));
    }
  }

  for (unsigned int i = 0; i < kNumOfMapVertices + kNumOfQueryVertices; ++i) {
    vi_map::Vertex& base_vertex = map_.getVertex(
        vertex_ids_[kNumOfMapVertices + kNumOfQueryVertices - 1 - i]);
    aslam::VisualNFrame& base_nframe = base_vertex.getVisualNFrame();
    const size_t num_frames = base_nframe.getNumFrames();
    vi_map::Vertex& vertex = submap_.getVertex(submap_vertex_ids_[i]);
    for (size_t frame_idx = 0; frame_idx < num_frames; ++frame_idx) {
      const size_t num_base_keypoints =
          base_vertex.observedLandmarkIdsSize(frame_idx);
      for (size_t observation_idx = 0; observation_idx < num_base_keypoints;
           ++observation_idx) {
        const vi_map::LandmarkId& base_landmark_id =
            base_vertex.getObservedLandmarkId(frame_idx, observation_idx);
        vi_map::Vertex& landmark_vertex =
            map_.getLandmarkStoreVertex(base_landmark_id);

        Eigen::Vector3d LM_p_fi =
            landmark_vertex.getLandmark_p_LM_fi(base_landmark_id);
        addKeypointToSubmapVertex(
            LM_p_fi, base_landmark_to_submap_id_map[base_landmark_id], &vertex);
      }
    }
  }
}

void LoopClosureHandlerTest::generateAndProjectLandmarksToMapKeyframes() {
  // All landmarks will be added to vertex_ids_[0] vertex for simplicity.
  vi_map::Vertex& landmark_vertex = map_.getVertex(vertex_ids_[0]);

  for (unsigned int i = 0; i < kNumOfLandmarks; ++i) {
    Eigen::Vector3d G_p_fi(0.25, 0, 10);
    G_p_fi += Eigen::Vector3d(dis_(gen_), dis_(gen_), dis_(gen_));

    vi_map::LandmarkId landmark_id;
    aslam::generateId(&landmark_id);

    addLandmarkToVertex(G_p_fi, landmark_id, &landmark_vertex);

    for (unsigned int j = 0; j < kNumOfMapVertices; ++j) {
      vi_map::Vertex& keypoint_vertex = map_.getVertex(vertex_ids_[j]);
      addKeypointToVertex(G_p_fi, landmark_id, &keypoint_vertex);
    }
  }
}

void LoopClosureHandlerTest::addLandmarkToVertex(
    const Eigen::Vector3d& G_p_fi, const vi_map::LandmarkId& landmark_id,
    vi_map::Vertex* vertex_ptr) {
  CHECK_NOTNULL(vertex_ptr);

  vi_map::Landmark::Ptr landmark_ptr(new vi_map::Landmark);
  landmark_ptr->setId(landmark_id);

  const Eigen::Vector3d& G_p_I = vertex_ptr->get_p_M_I();
  const Eigen::Quaterniond& G_q_I = vertex_ptr->get_q_M_I();

  const Eigen::Matrix3d I_R_G = G_q_I.toRotationMatrix().transpose();

  Eigen::Vector3d I_p_fi = I_R_G * (G_p_fi - G_p_I);
  landmark_ptr->set_p_B(pose::Position3D(I_p_fi));
  vertex_ptr->getLandmarks().addLandmark(*landmark_ptr);

  landmark_index_.addLandmarkAndVertexReference(
      landmark_ptr->id(), vertex_ptr->id());
}

void LoopClosureHandlerTest::addLandmarkToSubmapVertex(
    const Eigen::Vector3d& G_p_fi, const vi_map::LandmarkId& landmark_id,
    vi_map::Vertex* vertex_ptr) {
  CHECK_NOTNULL(vertex_ptr);

  vi_map::Landmark::Ptr landmark_ptr(new vi_map::Landmark);
  landmark_ptr->setId(landmark_id);

  const Eigen::Vector3d& G_p_I = vertex_ptr->get_p_M_I();
  const Eigen::Quaterniond& G_q_I = vertex_ptr->get_q_M_I();

  const Eigen::Matrix3d I_R_G = G_q_I.toRotationMatrix().transpose();

  Eigen::Vector3d I_p_fi = I_R_G * (G_p_fi - G_p_I);
  landmark_ptr->set_p_B(pose::Position3D(I_p_fi));
  vertex_ptr->getLandmarks().addLandmark(*landmark_ptr);

  submap_landmark_index_.addLandmarkAndVertexReference(
      landmark_ptr->id(), vertex_ptr->id());
}

unsigned int LoopClosureHandlerTest::addKeypointToVertex(
    const Eigen::Vector3d& G_p_fi, const vi_map::LandmarkId& landmark_id,
    vi_map::Vertex* vertex_ptr) {
  CHECK_NOTNULL(vertex_ptr);

  const Eigen::Vector3d& C_p_I =
      cameras_->get_T_C_B(kVisualFrameIndex).getPosition();
  const Eigen::Quaterniond& C_q_I =
      cameras_->get_T_C_B(kVisualFrameIndex).getRotation().toImplementation();
  const Eigen::Matrix3d C_R_I = C_q_I.toRotationMatrix();

  const Eigen::Vector3d& G_p_I = vertex_ptr->get_p_M_I();
  const Eigen::Quaterniond& G_q_I = vertex_ptr->get_q_M_I();
  const Eigen::Matrix3d I_R_G = G_q_I.toRotationMatrix().transpose();

  const Eigen::Vector3d I_p_fi = I_R_G * (G_p_fi - G_p_I);
  const Eigen::Vector3d C_p_fi = C_R_I * I_p_fi + C_p_I;

  Eigen::Vector2d reprojected_point;
  vertex_ptr->getCamera(kVisualFrameIndex)
      ->project3(C_p_fi, &reprojected_point);

  if (!vertex_ptr->getVisualFrame(kVisualFrameIndex)
           .hasKeypointMeasurements()) {
    Eigen::Matrix2Xd empty_measurements;
    empty_measurements.resize(Eigen::NoChange, 0);
    vertex_ptr->getVisualFrame(kVisualFrameIndex)
        .setKeypointMeasurements(empty_measurements);
  }
  CHECK(
      vertex_ptr->getVisualFrame(kVisualFrameIndex).hasKeypointMeasurements());

  Eigen::Matrix2Xd* measurements = vertex_ptr->getVisualFrame(kVisualFrameIndex)
                                       .getKeypointMeasurementsMutable();
  CHECK_NOTNULL(measurements);
  measurements->conservativeResize(Eigen::NoChange, measurements->cols() + 1);
  measurements->rightCols(1) = reprojected_point;

  addObservedLandmarkId(landmark_id, vertex_ptr);

  const unsigned int keypoint_idx =
      vertex_ptr->observedLandmarkIdsSize(kVisualFrameIndex) - 1;
  map_.getLandmark(landmark_id)
      .addObservation(vertex_ptr->id(), kVisualFrameIndex, keypoint_idx);

  return keypoint_idx;
}

unsigned int LoopClosureHandlerTest::addKeypointToSubmapVertex(
    const Eigen::Vector3d& G_p_fi, const vi_map::LandmarkId& landmark_id,
    vi_map::Vertex* vertex_ptr) {
  CHECK_NOTNULL(vertex_ptr);

  const Eigen::Vector3d& C_p_I =
      cameras_->get_T_C_B(kVisualFrameIndex).getPosition();
  const Eigen::Quaterniond& C_q_I =
      cameras_->get_T_C_B(kVisualFrameIndex).getRotation().toImplementation();
  const Eigen::Matrix3d C_R_I = C_q_I.toRotationMatrix();

  const Eigen::Vector3d& G_p_I = vertex_ptr->get_p_M_I();
  const Eigen::Quaterniond& G_q_I = vertex_ptr->get_q_M_I();
  const Eigen::Matrix3d I_R_G = G_q_I.toRotationMatrix().transpose();

  const Eigen::Vector3d I_p_fi = I_R_G * (G_p_fi - G_p_I);
  const Eigen::Vector3d C_p_fi = C_R_I * I_p_fi + C_p_I;

  Eigen::Vector2d reprojected_point;
  vertex_ptr->getCamera(kVisualFrameIndex)
      ->project3(C_p_fi, &reprojected_point);

  if (!vertex_ptr->getVisualFrame(kVisualFrameIndex)
           .hasKeypointMeasurements()) {
    Eigen::Matrix2Xd empty_measurements;
    empty_measurements.resize(Eigen::NoChange, 0);
    vertex_ptr->getVisualFrame(kVisualFrameIndex)
        .setKeypointMeasurements(empty_measurements);
  }
  CHECK(
      vertex_ptr->getVisualFrame(kVisualFrameIndex).hasKeypointMeasurements());

  Eigen::Matrix2Xd* measurements = vertex_ptr->getVisualFrame(kVisualFrameIndex)
                                       .getKeypointMeasurementsMutable();
  CHECK_NOTNULL(measurements);
  measurements->conservativeResize(Eigen::NoChange, measurements->cols() + 1);
  measurements->rightCols(1) = reprojected_point;

  addObservedLandmarkId(landmark_id, vertex_ptr);

  const unsigned int keypoint_idx =
      vertex_ptr->observedLandmarkIdsSize(kVisualFrameIndex) - 1;
  submap_.getLandmark(landmark_id)
      .addObservation(vertex_ptr->id(), kVisualFrameIndex, keypoint_idx);

  return keypoint_idx;
}

void LoopClosureHandlerTest::addDuplicateLandmarksToQueryKeyframes() {
  CHECK_GE(landmark_index_.numLandmarks(), kNumOfDuplicateLandmarks);

  vi_map::LandmarkIdList all_landmark_ids;
  landmark_index_.getAllLandmarkIds(&all_landmark_ids);

  vi_map::LandmarkIdSet landmarks_to_be_duplicated;
  for (const vi_map::LandmarkId& landmark_id : all_landmark_ids) {
    landmarks_to_be_duplicated.emplace(landmark_id);

    if (landmarks_to_be_duplicated.size() == kNumOfDuplicateLandmarks) {
      break;
    }
  }

  for (const vi_map::LandmarkId& landmark_id : landmarks_to_be_duplicated) {
    vi_map::Vertex& landmark_vertex = map_.getLandmarkStoreVertex(landmark_id);

    Eigen::Vector3d LM_p_fi = landmark_vertex.getLandmark_p_LM_fi(landmark_id);
    LM_p_fi +=
        Eigen::Vector3d(dis_(gen_) / 100, dis_(gen_) / 100, dis_(gen_) / 100);

    vi_map::LandmarkId new_landmark_id;
    aslam::generateId(&new_landmark_id);

    addLandmarkToVertex(LM_p_fi, new_landmark_id, &landmark_vertex);
    duplicate_landmark_to_landmark_map_.emplace(new_landmark_id, landmark_id);

    for (unsigned int i = kNumOfMapVertices; i < vertex_ids_.size(); ++i) {
      vi_map::Vertex& keypoint_vertex = map_.getVertex(vertex_ids_[i]);

      int keypoint_idx =
          addKeypointToVertex(LM_p_fi, new_landmark_id, &keypoint_vertex);

      ExpectedLandmarkMergeTriple expected_merge;
      expected_merge.vertex_id = vertex_ids_[i];
      expected_merge.idx = keypoint_idx;
      expected_merge.new_landmark_id = landmark_id;
      expected_landmark_merges_.push_back(expected_merge);
    }
  }

  for (unsigned int i = kNumOfMapVertices; i < vertex_ids_.size(); ++i) {
    vi_map::Vertex& query_vertex = map_.getVertex(vertex_ids_[i]);

    for (unsigned int j = 0; j < kNumOfMapVertices; ++j) {
      vi_map::LoopClosureConstraint constraint;
      constraint.query_vertex_id = vertex_ids_[i];

      const int num_keypoints =
          query_vertex.observedLandmarkIdsSize(kVisualFrameIndex);
      for (int keypoint_idx = 0; keypoint_idx < num_keypoints; ++keypoint_idx) {
        vi_map::LandmarkId duplicate_landmark_id =
            query_vertex.getObservedLandmarkId(kVisualFrameIndex, keypoint_idx);

        LandmarkToLandmarkMap::iterator it =
            duplicate_landmark_to_landmark_map_.find(duplicate_landmark_id);
        ASSERT_TRUE(it != duplicate_landmark_to_landmark_map_.end());
        const vi_map::Landmark& map_landmark = map_.getLandmark(it->second);

        for (const vi_map::KeypointIdentifier& observation :
             map_landmark.getObservations()) {
          if (observation.frame_id.vertex_id == vertex_ids_[j]) {
            vi_map::VertexKeyPointToStructureMatch match;
            match.frame_index_query = kVisualFrameIndex;
            match.keypoint_index_query = keypoint_idx;
            match.landmark_result = it->second;
            match.frame_identifier_result.frame_index = kVisualFrameIndex;
            match.frame_identifier_result.vertex_id =
                observation.frame_id.vertex_id;

            constraint.structure_matches.push_back(match);
            break;
          }
        }
      }
      constraints_.push_back(constraint);
    }
  }

  for (unsigned int j = kNumOfDuplicateLandmarks; j < kNumOfLandmarks; j++) {
    vi_map::Vertex& landmark_vertex =
        map_.getLandmarkStoreVertex(all_landmark_ids[j]);
    Eigen::Vector3d LM_p_fi =
        landmark_vertex.getLandmark_p_LM_fi(all_landmark_ids[j]);
    for (unsigned int i = kNumOfMapVertices; i < vertex_ids_.size(); ++i) {
      vi_map::Vertex& keypoint_vertex = map_.getVertex(vertex_ids_[i]);
      int keypoint_idx =
          addKeypointToVertex(LM_p_fi, all_landmark_ids[j], &keypoint_vertex);
    }
  }
}

bool LoopClosureHandlerTest::testLandmarkConsistency() {
  std::vector<vi_map::LandmarkId> landmark_ids;
  landmark_index_.getAllLandmarkIds(&landmark_ids);
  LOG(INFO) << landmark_ids.size();
  vi_map::LandmarkIdList all_landmarks_map;
  map_.getAllLandmarkIds(&all_landmarks_map);

  CHECK_EQ(all_landmarks_map.size(), landmark_ids.size());
  CHECK_EQ(landmark_ids.size(), kNumOfLandmarks);
  // Test if all landmarks in landmark_index exist in vertices in the map
  for (vi_map::LandmarkId landmark_id : landmark_ids) {
    CHECK(landmark_index_.hasLandmark(landmark_id));
    pose_graph::VertexId vertex_id =
        landmark_index_.getStoringVertexId(landmark_id);
    CHECK(map_.hasVertex(vertex_id));
    const vi_map::Vertex& vertex = map_.getVertex(vertex_id);
    CHECK(vertex.getLandmarks().hasLandmark(landmark_id));
  }
  pose_graph::VertexIdList all_vertex_ids;
  map_.getAllVertexIds(&all_vertex_ids);

  // Test if all vertices in the submap have been added to the base map
  CHECK_EQ(
      all_vertex_ids.size(),
      vertex_ids_.size() + submap_vertex_ids_.size() - 1);
  for (pose_graph::VertexId vertex_id : all_vertex_ids) {
    vi_map::Vertex& vertex = map_.getVertex(vertex_id);
    CHECK(vertex.hasVisualNFrame());
    aslam::VisualNFrame& nframe = vertex.getVisualNFrame();
    const size_t num_frames = nframe.getNumFrames();
    for (size_t frame_idx = 0; frame_idx < num_frames; ++frame_idx) {
      const size_t num_keypoints = vertex.observedLandmarkIdsSize(frame_idx);
      // Test if all keypoint observations are associated to a landmark in the
      // landmark_index and that these landmarks are stored by
      // vertices in the map
      for (size_t observation_idx = 0; observation_idx < num_keypoints;
           ++observation_idx) {
        const vi_map::LandmarkId& landmark_id =
            vertex.getObservedLandmarkId(frame_idx, observation_idx);
        CHECK(hasLandmark(landmark_id));
        vi_map::Vertex& landmark_vertex =
            map_.getLandmarkStoreVertex(landmark_id);
        CHECK(landmark_vertex.hasStoredLandmark(landmark_id));
        vi_map::Landmark landmark =
            landmark_vertex.getLandmarks().getLandmark(landmark_id);
        // Test if keypoint observation is stored in landmark
        CHECK(landmark.hasObservation(vertex_id, frame_idx, observation_idx));
      }
    }

    // Test if all landmarks stored in any vertex are in the landmark_index
    // and if they have observations
    std::vector<vi_map::LandmarkId> stored_landmark_ids;
    vertex.getStoredLandmarkIdList(&stored_landmark_ids);
    for (vi_map::LandmarkId landmark_id : stored_landmark_ids) {
      CHECK(vertex.getLandmarks().hasLandmark(landmark_id));
      CHECK(hasLandmark(landmark_id));
      vi_map::Landmark landmark =
          vertex.getLandmarks().getLandmark(landmark_id);
      const vi_map::KeypointIdentifierList& observations =
          landmark.getObservations();
      CHECK(!observations.empty());
    }
  }

  return true;
}

TEST_F(LoopClosureHandlerTest, LoopClosureHandlingTest) {
  int num_inliers;
  double inlier_ratio;
  static constexpr bool kMergeLandmarks = true;
  static constexpr bool kAddLoopClosureEdegs = false;
  FLAGS_lc_ransac_pixel_sigma = 0.8;

  std::mutex map_mutex;
  for (const vi_map::LoopClosureConstraint& constraint : constraints_) {
    pose::Transformation G_T_I;
    vi_map::LoopClosureConstraint inlier_constraints;
    loop_closure_handler::LoopClosureHandler::MergedLandmark3dPositionVector
        landmark_pairs_merged;
    constexpr pose_graph::VertexId* kVertexIdClosestToStructureMatches =
        nullptr;
    handler_->handleLoopClosure(
        constraint, kMergeLandmarks, kAddLoopClosureEdegs, &num_inliers,
        &inlier_ratio, &G_T_I, &inlier_constraints, &landmark_pairs_merged,
        kVertexIdClosestToStructureMatches, &map_mutex);

    EXPECT_GT(num_inliers, 0);
    EXPECT_GT(inlier_ratio, 0);

    EXPECT_EQ(kNumOfDuplicateLandmarks, expected_landmark_merges_.size());
  }

  for (const ExpectedLandmarkMergeTriple& expected_merge :
       expected_landmark_merges_) {
    vi_map::Vertex& query_vertex = map_.getVertex(expected_merge.vertex_id);

    EXPECT_EQ(
        expected_merge.new_landmark_id,
        query_vertex.getObservedLandmarkId(
            kVisualFrameIndex, expected_merge.idx));
  }

  // Check if deleted landmarks are really removed from global landmark to
  // vertex map.
  for (const LandmarkToLandmarkMap::value_type& old_landmark_to_landmark :
       duplicate_landmark_to_landmark_map_) {
    EXPECT_FALSE(hasLandmark(old_landmark_to_landmark.first));
  }

  // Also check if all the landmarks are still available.
  for (unsigned int i = 0; i < kNumOfMapVertices + kNumOfQueryVertices; ++i) {
    vi_map::Vertex& vertex = map_.getVertex(vertex_ids_[i]);
    const int num_of_landmarks =
        vertex.observedLandmarkIdsSize(kVisualFrameIndex);
    for (int j = 0; j < num_of_landmarks; ++j) {
      vi_map::LandmarkId landmark_id =
          vertex.getObservedLandmarkId(kVisualFrameIndex, j);

      EXPECT_TRUE(hasLandmark(landmark_id));
    }
  }
}

TEST_F(LoopClosureHandlerTest, SubmapMergingTest) {
  int num_inliers;
  double inlier_ratio;
  static constexpr bool kMergeLandmarks = true;
  static constexpr bool kAddLoopClosureEdegs = false;
  FLAGS_lc_ransac_pixel_sigma = 0.8;

  std::mutex map_mutex;
  for (const vi_map::LoopClosureConstraint& constraint : constraints_) {
    pose::Transformation G_T_I;
    vi_map::LoopClosureConstraint inlier_constraints;
    loop_closure_handler::LoopClosureHandler::MergedLandmark3dPositionVector
        landmark_pairs_merged;
    constexpr pose_graph::VertexId* kVertexIdClosestToStructureMatches =
        nullptr;
    handler_->handleLoopClosure(
        constraint, kMergeLandmarks, kAddLoopClosureEdegs, &num_inliers,
        &inlier_ratio, &G_T_I, &inlier_constraints, &landmark_pairs_merged,
        kVertexIdClosestToStructureMatches, &map_mutex);

    EXPECT_GT(num_inliers, 0);
    EXPECT_GT(inlier_ratio, 0);

    EXPECT_EQ(kNumOfDuplicateLandmarks, expected_landmark_merges_.size());
  }

  populateSubmap();
  map_.mergeAllSubmapsFromMap(submap_);
  CHECK(testLandmarkConsistency());

  // for (const ExpectedLandmarkMergeTriple& expected_merge :
  //      expected_landmark_merges_) {
  //   vi_map::Vertex& query_vertex = map_.getVertex(expected_merge.vertex_id);
  //
  //   EXPECT_EQ(
  //       expected_merge.new_landmark_id,
  //       query_vertex.getObservedLandmarkId(
  //               kVisualFrameIndex, expected_merge.idx));
  // }
  //
  // // Check if deleted landmarks are really removed from global landmark to
  // // vertex map.
  // for (const LandmarkToLandmarkMap::value_type& old_landmark_to_landmark :
  //      duplicate_landmark_to_landmark_map_) {
  //   EXPECT_FALSE(hasLandmark(old_landmark_to_landmark.first));
  // }
  //
  // // Also check if all the landmarks are still available.
  // for (unsigned int i = 0; i < kNumOfMapVertices + kNumOfQueryVertices; ++i)
  // {
  //   vi_map::Vertex& vertex = map_.getVertex(vertex_ids_[i]);
  //   const int num_of_landmarks =
  //       vertex.observedLandmarkIdsSize(kVisualFrameIndex);
  //   for (int j = 0; j < num_of_landmarks; ++j) {
  //     vi_map::LandmarkId landmark_id =
  //         vertex.getObservedLandmarkId(kVisualFrameIndex, j);
  //
  //     EXPECT_TRUE(hasLandmark(landmark_id));
  //   }
  // }
}

MAPLAB_UNITTEST_ENTRYPOINT
