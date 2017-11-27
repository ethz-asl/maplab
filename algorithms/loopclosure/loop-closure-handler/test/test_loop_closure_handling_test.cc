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
        new loop_closure_handler::LoopClosureHandler(&map_,
                                                     &landmark_id_old_to_new_));
    CHECK_LE(kNumOfDuplicateLandmarks, kNumOfLandmarks);
  }

  bool hasLandmark(const vi_map::LandmarkId& landmark_id) const;

  pose_graph::VertexId getVertexIdForLandmark(
      const vi_map::LandmarkId& landmark_id) const;

  void constructCamera();
  void createMission();
  void populatePosegraph();
  void generateAndProjectLandmarksToMapKeyframes();
  unsigned int addKeypointToVertex(
      const Eigen::Vector3d& G_p_fi,
      const vi_map::LandmarkId& landmark_id,
      vi_map::Vertex* vertex_ptr);
  void addLandmarkToVertex(
      const Eigen::Vector3d& G_p_fi, const vi_map::LandmarkId& landmark_id,
      vi_map::Vertex* vertex_ptr);
  void addDuplicateLandmarksToQueryKeyframes();

  vi_map::VIMap map_;
  vi_map::PoseGraph& posegraph_;
  vi_map::LandmarkIndex& landmark_index_;
  vi_map::VIMissionMap& missions_;
  vi_map::MissionBaseFrameMap& mission_base_frames_;

  std::shared_ptr<loop_closure_handler::LoopClosureHandler> handler_;
  LandmarkToLandmarkMap landmark_id_old_to_new_;

  std::vector<pose_graph::VertexId> vertex_ids_;
  vi_map::MissionId mission_id_;
  aslam::NCamera::Ptr cameras_;
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
  common::generateId(&camera_id);
  camera->setId(camera_id);

  std::vector<aslam::Camera::Ptr> camera_vector;
  camera_vector.push_back(camera);
  aslam::TransformationVector T_C_B_vector;
  aslam::Transformation T_C_B(
      Eigen::Quaterniond(sqrt(2) / 2, 0, sqrt(2) / 2, 0),
      Eigen::Vector3d(1, 2, 3));
  T_C_B_vector.push_back(T_C_B);
  aslam::NCameraId n_camera_id;
  common::generateId(&n_camera_id);
  cameras_.reset(
      new aslam::NCamera(
          n_camera_id, T_C_B_vector, camera_vector, "Test camera rig"));
}

void LoopClosureHandlerTest::createMission() {
  vi_map::MissionBaseFrame baseframe;
  vi_map::MissionBaseFrameId baseframe_id;
  common::generateId(&baseframe_id);
  baseframe.setId(baseframe_id);

  baseframe.set_p_G_M(Eigen::Matrix<double, 3, 1>::Zero());
  baseframe.set_q_G_M(Eigen::Quaterniond::Identity());

  vi_map::VIMission::UniquePtr mission_ptr(new vi_map::VIMission);

  common::generateId(&mission_id_);
  mission_ptr->setId(mission_id_);

  mission_ptr->setBaseFrameId(baseframe_id);

  missions_.emplace(mission_ptr->id(), std::move(mission_ptr));
  mission_base_frames_.emplace(baseframe.id(), baseframe);
}

void LoopClosureHandlerTest::populatePosegraph() {
  Eigen::Quaterniond orientation(1, 0, 0, 0);
  Eigen::Vector3d position(0, 0, 0);

  pose_graph::VertexId vertex_id;
  vi_map::Vertex::UniquePtr vertex(new vi_map::Vertex(cameras_));

  common::generateId(&vertex_id);
  vertex->setId(vertex_id);
  vertex->set_p_M_I(position);
  vertex->set_q_M_I(orientation);
  vertex->setMissionId(mission_id_);
  posegraph_.addVertex(std::move(vertex));

  vertex_ids_.resize(kNumOfMapVertices + kNumOfQueryVertices);
  vertex_ids_[0] = vertex_id;

  for (unsigned int i = 1; i < kNumOfMapVertices + kNumOfQueryVertices; ++i) {
    vi_map::Vertex::UniquePtr vertex(new vi_map::Vertex(cameras_));
    common::generateId(&vertex_id);
    vertex->setId(vertex_id);
    position.x() += i * 0.2;
    vertex->set_p_M_I(position);
    vertex->set_q_M_I(orientation);
    vertex->setMissionId(mission_id_);
    posegraph_.addVertex(std::move(vertex));
    vertex_ids_[i] = vertex_id;

    pose_graph::EdgeId edge_id;
    common::generateId(&edge_id);

    Eigen::Matrix<int64_t, 1, Eigen::Dynamic> imu_timestamps;
    Eigen::Matrix<double, 6, Eigen::Dynamic> imu_data;
    vi_map::MissionId mission_id;
    common::generateId(&mission_id);

    vi_map::ViwlsEdge::UniquePtr edge(
        new vi_map::ViwlsEdge(
            edge_id, vertex_ids_[i], vertex_ids_[i - 1], imu_timestamps,
            imu_data));
    posegraph_.addEdge(std::move(edge));
  }
}

void LoopClosureHandlerTest::generateAndProjectLandmarksToMapKeyframes() {
  // All landmarks will be added to vertex_ids_[0] vertex for simplicity.
  vi_map::Vertex& landmark_vertex = map_.getVertex(vertex_ids_[0]);

  for (unsigned int i = 0; i < kNumOfLandmarks; ++i) {
    Eigen::Vector3d G_p_fi(0.25, 0, 10);
    G_p_fi += Eigen::Vector3d(dis_(gen_), dis_(gen_), dis_(gen_));

    vi_map::LandmarkId landmark_id;
    common::generateId(&landmark_id);

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

unsigned int LoopClosureHandlerTest::addKeypointToVertex(
    const Eigen::Vector3d& G_p_fi,
    const vi_map::LandmarkId& landmark_id,
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
  map_.getLandmark(landmark_id).addObservation(vertex_ptr->id(),
                                               kVisualFrameIndex, keypoint_idx);

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
    vi_map::Vertex& landmark_vertex =
        map_.getLandmarkStoreVertex(landmark_id);

    Eigen::Vector3d LM_p_fi = landmark_vertex.getLandmark_p_LM_fi(landmark_id);
    LM_p_fi +=
        Eigen::Vector3d(dis_(gen_) / 100, dis_(gen_) / 100, dis_(gen_) / 100);

    vi_map::LandmarkId new_landmark_id;
    common::generateId(&new_landmark_id);

    addLandmarkToVertex(LM_p_fi, new_landmark_id, &landmark_vertex);
    duplicate_landmark_to_landmark_map_.emplace(
        new_landmark_id, landmark_id);

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
        const vi_map::Landmark& map_landmark =
            map_.getLandmark(it->second);

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

MAPLAB_UNITTEST_ENTRYPOINT
