#include <memory>

#include <Eigen/Core>

#include <map-sparsification/heuristic/cost-functions/min-keypoints-per-keyframe-cost.h>
#include <map-sparsification/heuristic/heuristic-sampling.h>
#include <map-sparsification/heuristic/scoring/descriptor-variance-scoring.h>
#include <map-sparsification/heuristic/scoring/observation-count-scoring.h>
#include <maplab-common/test/testing-entrypoint.h>
#include <maplab-common/test/testing-predicates.h>

#include <vi-map/pose-graph.h>
#include <vi-map/test/vi-map-generator.h>
#include <vi-map/vi-map.h>

class MapSparsification : public testing::Test {
 protected:
  typedef map_sparsification::sampling::LandmarkSamplingWithCostFunctions
      Sampler;

  virtual void SetUp() {
    generator_map_.reset(new vi_map::VIMap);
    constexpr int kSeed = 42;
    generator_.reset(new vi_map::VIMapGenerator(*generator_map_, kSeed));

    is_map_data_initialized_ = false;

    initializeMapData();
    addMissionAndVertices();
    buildMapSummarizer();
  }

  const vi_map::LandmarkId& generateMapToTestObservationCount();
  const vi_map::LandmarkId& generateMapToTestKeyframeConstraints();
  void sample(vi_map::LandmarkIdSet* summary_store_landmark_ids);

  void expectRemovedLandmark(
      const vi_map::LandmarkIdSet& summary_store_landmark_ids,
      const vi_map::LandmarkId& deleted_landmark) const;
  void expectPreservedLandmark(
      const vi_map::LandmarkIdSet& summary_store_landmark_ids,
      const vi_map::LandmarkId& preserved_landmark) const;

  inline int getNumDesiredLandmarks() const {
    return kNumDesiredNumLandmarks;
  }

 private:
  void initializeMapData();
  void addMissionAndVertices();
  void buildMapSummarizer();

  std::unique_ptr<vi_map::VIMap> generator_map_;
  std::unique_ptr<vi_map::VIMapGenerator> generator_;
  Sampler sampler_;

  static constexpr int kNumVertices = 5;
  static constexpr int kNumLandmarks = 5;
  static constexpr int kNumMissions = 1;
  static constexpr int kNumDesiredNumLandmarks = 4;

  vi_map::MissionId mission_id_;
  pose_graph::VertexId vertex_ids_[kNumVertices];
  vi_map::LandmarkId landmark_ids_[kNumLandmarks];
  pose::Transformation T_G_M_;
  pose::Transformation T_G_I_[kNumVertices];
  Eigen::Vector3d p_G_fi_[kNumLandmarks];

  bool is_map_data_initialized_;

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

void MapSparsification::initializeMapData() {
  ASSERT_FALSE(is_map_data_initialized_);

  T_G_M_.setIdentity();

  for (unsigned int i = 0; i < kNumVertices; ++i) {
    T_G_I_[i].setIdentity();
  }

  for (unsigned int i = 0; i < kNumLandmarks; ++i) {
    p_G_fi_[i] = Eigen::Vector3d(1, 1, 1);
  }
  is_map_data_initialized_ = true;
}

void MapSparsification::addMissionAndVertices() {
  ASSERT_TRUE(is_map_data_initialized_);

  mission_id_ = generator_->createMission(T_G_M_);

  for (unsigned int i = 0; i < kNumVertices; ++i) {
    vertex_ids_[i] = generator_->createVertex(mission_id_, T_G_I_[i]);
  }
}

// This method generates a map where one of the landmarks has fewer
// observations than all the others. ID of this landmark is returned.
const vi_map::LandmarkId&
MapSparsification::generateMapToTestObservationCount() {
  pose_graph::VertexIdList all_vertices;
  all_vertices.assign(vertex_ids_ + 1, vertex_ids_ + kNumVertices);
  for (unsigned int i = 1; i < kNumLandmarks; ++i) {
    landmark_ids_[i] =
        generator_->createLandmark(p_G_fi_[i], vertex_ids_[0], all_vertices);
  }

  pose_graph::VertexIdList empty_vertex_list;
  landmark_ids_[0] =
      generator_->createLandmark(p_G_fi_[0], vertex_ids_[0], empty_vertex_list);
  generator_->generateMap();
  return landmark_ids_[0];
}

// This method generates a map where one of the keyframes is observed by
// smaller number of landmarks than the rest. The observation count of all
// landmarks is identical. The method returns the landmark ID that should
// be preserved because of keypoint per keyframe threshold violation.
const vi_map::LandmarkId&
MapSparsification::generateMapToTestKeyframeConstraints() {
  const pose_graph::VertexId additional_vertex =
      generator_->createVertex(mission_id_, T_G_I_[0]);

  pose_graph::VertexIdList vertices_minus_first_and_last;
  vertices_minus_first_and_last.assign(
      vertex_ids_ + 1, vertex_ids_ + kNumVertices - 1);
  for (unsigned int i = 1; i < kNumLandmarks; ++i) {
    landmark_ids_[i] = generator_->createLandmark(
        p_G_fi_[i], vertex_ids_[0], vertices_minus_first_and_last);
  }

  pose_graph::VertexIdList vertices_minus_first_plus_additional;
  vertices_minus_first_plus_additional.assign(
      vertex_ids_ + 1, vertex_ids_ + kNumVertices);
  vertices_minus_first_plus_additional.push_back(additional_vertex);

  // Additional link s.t. all landmarks have idential number of observers.
  landmark_ids_[0] = generator_->createLandmark(
      p_G_fi_[0], vertex_ids_[0], vertices_minus_first_plus_additional);

  generator_->generateMap();
  return landmark_ids_[0];
}

void MapSparsification::buildMapSummarizer() {
  using map_sparsification::cost_functions::IsRequiredToConstrainKeyframesCost;
  using map_sparsification::scoring::ObservationCountScoringFunction;
  using map_sparsification::scoring::DescriptorVarianceScoring;

  const double kDescriptorDevThreshold = 30.0;
  const int kMinKeyframesPerKeypoint = 5;

  DescriptorVarianceScoring::Ptr descriptor_dev_score(
      new DescriptorVarianceScoring(kDescriptorDevThreshold));
  IsRequiredToConstrainKeyframesCost::Ptr keyframe_keypoint_cost(
      new IsRequiredToConstrainKeyframesCost(kMinKeyframesPerKeypoint));
  ObservationCountScoringFunction::Ptr obs_count_score(
      new ObservationCountScoringFunction);

  const double kDescriptorVarianceWeight = 0.2;
  const double kObsCountWeight = 1.0;
  const double kKeyframeConstraintWeight = 1.0;
  descriptor_dev_score->setWeight(kDescriptorVarianceWeight);
  obs_count_score->setWeight(kObsCountWeight);
  keyframe_keypoint_cost->setWeight(kKeyframeConstraintWeight);

  // Squared loss.
  std::function<double(double)> keyframe_constraint_loss =  // NOLINT
      [](double x) {
        const double kLinearFactor = 5.0;
        return kLinearFactor * x * x;
      };  // NOLINT
  keyframe_keypoint_cost->setLossFunction(keyframe_constraint_loss);

  sampler_.registerScoringFunction(obs_count_score);
  sampler_.registerScoringFunction(descriptor_dev_score);
  sampler_.registerCostFunction(keyframe_keypoint_cost);
}

void MapSparsification::sample(
    vi_map::LandmarkIdSet* summary_store_landmark_ids) {
  CHECK_NOTNULL(summary_store_landmark_ids);
  sampler_.sample(
      *generator_map_, kNumDesiredNumLandmarks, summary_store_landmark_ids);
}

void MapSparsification::expectRemovedLandmark(
    const vi_map::LandmarkIdSet& summary_landmark_ids,
    const vi_map::LandmarkId& deleted_landmark) const {
  EXPECT_EQ(0u, summary_landmark_ids.count(deleted_landmark));
  for (unsigned int i = 0; i < kNumLandmarks; ++i) {
    if (landmark_ids_[i] != deleted_landmark) {
      EXPECT_EQ(1u, summary_landmark_ids.count(landmark_ids_[i]));
    }
  }
}

void MapSparsification::expectPreservedLandmark(
    const vi_map::LandmarkIdSet& summary_landmark_ids,
    const vi_map::LandmarkId& preserved_landmark) const {
  // Expect that preserved_landmark is indeed preserved.
  EXPECT_EQ(1u, summary_landmark_ids.count(preserved_landmark));
  int num_preserved_landmarks = 0;
  for (unsigned int i = 0; i < kNumLandmarks; ++i) {
    if (summary_landmark_ids.count(landmark_ids_[i]) > 0u) {
      ++num_preserved_landmarks;
    }
  }
  // Other landmarks could be deleted, but the total number has to match
  // the desired number.
  EXPECT_EQ(getNumDesiredLandmarks(), num_preserved_landmarks);
}

TEST_F(MapSparsification, ObservationCountSparsificationTest) {
  const vi_map::LandmarkId& landmark_to_be_deleted =
      generateMapToTestObservationCount();

  vi_map::LandmarkIdSet summary_landmark_list;
  sample(&summary_landmark_list);

  expectRemovedLandmark(summary_landmark_list, landmark_to_be_deleted);
  EXPECT_EQ(getNumDesiredLandmarks(), summary_landmark_list.size());
}

TEST_F(MapSparsification, KeyframeKeypointConstraintSparsificationTest) {
  const vi_map::LandmarkId& landmark_to_be_preserved =
      generateMapToTestKeyframeConstraints();

  vi_map::LandmarkIdSet summary_landmark_list;
  sample(&summary_landmark_list);

  expectPreservedLandmark(summary_landmark_list, landmark_to_be_preserved);
}

MAPLAB_UNITTEST_ENTRYPOINT
