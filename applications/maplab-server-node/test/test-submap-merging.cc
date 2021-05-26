#include <loop-closure-handler/test/test_loop_closure_handling_test.h>

static constexpr unsigned int kNumOfInvalidLastBaseVertexObservation = 5;
static constexpr unsigned int kNumOfInvalidFirstSubmapVertexObservation = 5;
static constexpr unsigned int kNumOfAdditionalSubmapLandmarks = 100;

class SubmapMergingTest : public ::LoopClosureHandlerTest {
 protected:
  SubmapMergingTest()
      : submap_(),
        submap_posegraph_(submap_.posegraph),
        submap_landmark_index_(submap_.landmark_index),
        submap_missions_(submap_.missions),
        submap_mission_base_frames_(submap_.mission_base_frames) {}

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

  void populateSubmap();
  void setFirstNObservationsInvalid(
      const pose_graph::VertexId vertex_id,
      const size_t num_invalid_observations, vi_map::VIMap* map);
  unsigned int addKeypointToSubmapVertex(
      const Eigen::Vector3d& G_p_fi, const vi_map::LandmarkId& landmark_id,
      vi_map::Vertex* vertex_ptr);
  void addLandmarkToSubmapVertex(
      const Eigen::Vector3d& G_p_fi, const vi_map::LandmarkId& landmark_id,
      vi_map::Vertex* vertex_ptr);

  void testLandmarkConsistency(const size_t n_expected_landmarks);
  void generateAndProjectLandmarksToSubmapVertex(vi_map::Vertex* vertex);

  vi_map::VIMap submap_;
  vi_map::PoseGraph& submap_posegraph_;
  vi_map::LandmarkIndex& submap_landmark_index_;
  vi_map::VIMissionMap& submap_missions_;
  vi_map::MissionBaseFrameMap& submap_mission_base_frames_;
  std::vector<pose_graph::VertexId> submap_vertex_ids_;

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

void SubmapMergingTest::populateSubmap() {
  // Creates a submap from the basemap but in reverse order of vertices
  submap_.getSensorManager().addSensor<aslam::NCamera>(
      aligned_unique<aslam::NCamera>(*cameras_), cameras_->getId(), T_C_B_);

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

void SubmapMergingTest::generateAndProjectLandmarksToSubmapVertex(
    vi_map::Vertex* vertex) {
  for (unsigned int i = 0; i < kNumOfAdditionalSubmapLandmarks; ++i) {
    Eigen::Vector3d G_p_fi(5.25, 0, 10);
    G_p_fi += Eigen::Vector3d(dis_(gen_), dis_(gen_), dis_(gen_));

    vi_map::LandmarkId landmark_id;
    aslam::generateId(&landmark_id);

    addLandmarkToSubmapVertex(G_p_fi, landmark_id, vertex);
    addKeypointToSubmapVertex(G_p_fi, landmark_id, vertex);
  }
}

void SubmapMergingTest::addLandmarkToSubmapVertex(
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

unsigned int SubmapMergingTest::addKeypointToSubmapVertex(
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

void SubmapMergingTest::setFirstNObservationsInvalid(
    const pose_graph::VertexId vertex_id, const size_t num_invalid_observations,
    vi_map::VIMap* map) {
  CHECK(map->hasVertex(vertex_id));
  vi_map::Vertex& vertex = map->getVertex(vertex_id);
  vi_map::LandmarkIdList observed_landmark_ids;
  vertex.getAllObservedLandmarkIds(&observed_landmark_ids);
  std::unordered_set<vi_map::LandmarkId> unique_landmark_ids(
      observed_landmark_ids.begin(), observed_landmark_ids.end());
  size_t num_invalid_ids;
  for (const vi_map::LandmarkId& landmark_id : unique_landmark_ids) {
    const pose_graph::VertexId& storing_vertex_id =
        map->landmark_index.getStoringVertexId(landmark_id);
    vi_map::Vertex& storing_vertex = map->getVertex(storing_vertex_id);
    vi_map::Landmark& landmark =
        storing_vertex.getLandmarks().getLandmark(landmark_id);
    landmark.removeAllObservationsOfVertex(vertex_id);
    if (landmark.getObservations().empty()) {
      map->landmark_index.removeLandmark(landmark_id);
      storing_vertex.getLandmarks().removeLandmark(landmark_id);
    }
    vi_map::LandmarkId invalid_id;
    vertex.updateIdInObservedLandmarkIdList(landmark_id, invalid_id);
    num_invalid_ids++;
    if (num_invalid_ids == num_invalid_observations) {
      break;
    }
  }
}

void SubmapMergingTest::testLandmarkConsistency(
    const size_t n_expected_landmarks) {
  std::vector<vi_map::LandmarkId> landmark_ids;
  landmark_index_.getAllLandmarkIds(&landmark_ids);
  vi_map::LandmarkIdList all_landmarks_map;
  map_.getAllLandmarkIds(&all_landmarks_map);

  // Test if map and landmark_index have same amount of landmarks
  EXPECT_EQ(all_landmarks_map.size(), landmark_ids.size());
  // Test if landmark_index has expected amount of landmarks
  EXPECT_EQ(landmark_ids.size(), n_expected_landmarks);
  // Test if all landmarks in landmark_index exist in vertices in the map
  for (vi_map::LandmarkId landmark_id : landmark_ids) {
    ASSERT_TRUE(landmark_index_.hasLandmark(landmark_id));
    pose_graph::VertexId vertex_id =
        landmark_index_.getStoringVertexId(landmark_id);
    ASSERT_TRUE(map_.hasVertex(vertex_id));
    const vi_map::Vertex& vertex = map_.getVertex(vertex_id);
    ASSERT_TRUE(vertex.getLandmarks().hasLandmark(landmark_id));
    vi_map::Landmark landmark = vertex.getLandmarks().getLandmark(landmark_id);
    vi_map::KeypointIdentifierList keypoint_identifiers =
        landmark.getObservations();

    // Test if all observations of landmark are also present in
    // vertex observations
    for (vi_map::KeypointIdentifier keypoint_identifier :
         keypoint_identifiers) {
      pose_graph::VertexId observing_vertex_id =
          keypoint_identifier.frame_id.vertex_id;
      ASSERT_TRUE(map_.hasVertex(observing_vertex_id));
      const vi_map::Vertex& observing_vertex =
          map_.getVertex(observing_vertex_id);
      ASSERT_TRUE(
          observing_vertex.getObservedLandmarkId(
              keypoint_identifier.frame_id.frame_index,
              keypoint_identifier.keypoint_index) == landmark_id);
    }
  }

  // Test if all vertices in the submap have been added to the base map
  pose_graph::VertexIdList all_vertex_ids;
  map_.getAllVertexIds(&all_vertex_ids);
  EXPECT_EQ(
      all_vertex_ids.size(),
      vertex_ids_.size() + submap_vertex_ids_.size() - 1);

  for (pose_graph::VertexId vertex_id : all_vertex_ids) {
    vi_map::Vertex& vertex = map_.getVertex(vertex_id);
    ASSERT_TRUE(vertex.hasVisualNFrame());
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
        ASSERT_TRUE(hasLandmark(landmark_id));
        vi_map::Vertex& landmark_vertex =
            map_.getLandmarkStoreVertex(landmark_id);
        ASSERT_TRUE(landmark_vertex.hasStoredLandmark(landmark_id));
        vi_map::Landmark landmark =
            landmark_vertex.getLandmarks().getLandmark(landmark_id);
        // Test if keypoint observation is stored in landmark
        EXPECT_TRUE(
            landmark.hasObservation(vertex_id, frame_idx, observation_idx));
      }
    }

    // Test if all landmarks stored in any vertex are in the landmark_index
    // and if they have observations
    std::vector<vi_map::LandmarkId> stored_landmark_ids;
    vertex.getStoredLandmarkIdList(&stored_landmark_ids);
    for (vi_map::LandmarkId landmark_id : stored_landmark_ids) {
      EXPECT_TRUE(hasLandmark(landmark_id));
      ASSERT_TRUE(vertex.getLandmarks().hasLandmark(landmark_id));
      vi_map::Landmark landmark =
          vertex.getLandmarks().getLandmark(landmark_id);
      const vi_map::KeypointIdentifierList& observations =
          landmark.getObservations();
      EXPECT_TRUE(!observations.empty());
    }
  }
}

TEST_F(SubmapMergingTest, SubmapMergingMergedLandmarksTest) {
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
  testLandmarkConsistency(kNumOfLandmarks);
}

TEST_F(SubmapMergingTest, SubmapMergingInvalidBasemapObservationsTest) {
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
  pose_graph::VertexId last_base_vertex_id =
      vertex_ids_[vertex_ids_.size() - 1u];
  setFirstNObservationsInvalid(
      last_base_vertex_id, kNumOfInvalidLastBaseVertexObservation, &map_);
  map_.mergeAllSubmapsFromMap(submap_);
  testLandmarkConsistency(
      kNumOfLandmarks + kNumOfInvalidLastBaseVertexObservation);
}

TEST_F(SubmapMergingTest, SubmapMergingAllInvalidBasemapObservationsTest) {
  populateSubmap();
  pose_graph::VertexId last_base_vertex_id =
      vertex_ids_[vertex_ids_.size() - 1u];
  vi_map::LandmarkIdList all_observations_last_base_vertex;
  map_.getVertex(last_base_vertex_id)
      .getAllObservedLandmarkIds(&all_observations_last_base_vertex);
  size_t n_last_base_vertex_observations =
      all_observations_last_base_vertex.size();
  setFirstNObservationsInvalid(
      last_base_vertex_id, n_last_base_vertex_observations, &map_);
  map_.mergeAllSubmapsFromMap(submap_);

  testLandmarkConsistency(2 * kNumOfLandmarks + kNumOfDuplicateLandmarks);
}

TEST_F(SubmapMergingTest, SubmapMergingInvalidSubmapObservationsTest) {
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
  pose_graph::VertexId first_submap_vertex_id = submap_vertex_ids_[0];
  setFirstNObservationsInvalid(
      first_submap_vertex_id, kNumOfInvalidFirstSubmapVertexObservation,
      &submap_);
  map_.mergeAllSubmapsFromMap(submap_);
  testLandmarkConsistency(
      kNumOfLandmarks + kNumOfInvalidFirstSubmapVertexObservation);
}

TEST_F(SubmapMergingTest, SubmapMergingTest) {
  populateSubmap();
  map_.mergeAllSubmapsFromMap(submap_);

  testLandmarkConsistency(kNumOfLandmarks + 2 * kNumOfDuplicateLandmarks);
}

TEST_F(SubmapMergingTest, SubmapMergingAdditionalSubmapLandmarksTest) {
  populateSubmap();
  vi_map::Vertex& vertex =
      submap_.getVertex(submap_vertex_ids_[submap_vertex_ids_.size() / 2]);
  generateAndProjectLandmarksToSubmapVertex(&vertex);
  map_.mergeAllSubmapsFromMap(submap_);
  testLandmarkConsistency(
      kNumOfLandmarks + 2 * kNumOfDuplicateLandmarks +
      kNumOfAdditionalSubmapLandmarks);
}

MAPLAB_UNITTEST_ENTRYPOINT
