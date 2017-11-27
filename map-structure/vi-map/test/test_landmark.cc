#include <vector>

#include <aslam/common/memory.h>
#include <glog/logging.h>
#include <gtest/gtest.h>
#include <maplab-common/test/testing-entrypoint.h>
#include <maplab-common/unique-id.h>
#include <posegraph/unique-id.h>

#include "vi-map/landmark.h"
#include "vi-map/unique-id.h"
#include "vi-map/vi_map.pb.h"

namespace vi_map {

class LandmarkTest : public ::testing::Test {
 protected:
  virtual void SetUp() {}

  void generateObservations(
      size_t num_observations_to_add, KeypointIdentifierList* observations) {
    CHECK_NOTNULL(observations)->clear();

    for (size_t idx = 0u; idx < num_observations_to_add; ++idx) {
      KeypointIdentifier observation;
      common::generateId(&observation.frame_id.vertex_id);
      observation.frame_id.frame_index = idx % 2u;
      observation.keypoint_index = idx;

      observations->emplace_back(observation);
    }
  }

  void allocateIncrementalAppearances() {
    landmark_.allocateAppearances();
    const size_t num_observations = landmark_.numberOfObservations();
    for (size_t idx = 0u; idx < num_observations; ++idx) {
      landmark_.setAppearance(idx, static_cast<int>(idx));
    }
  }

  Landmark landmark_;
};

bool verifyIncrementalAppearances(const Landmark& landmark) {
  bool valid = true;

  const size_t num_observations = landmark.numberOfObservations();

  std::unordered_set<int> distinct_appearances;
  landmark.getAllDistinctAppearances(&distinct_appearances);

  valid &= distinct_appearances.size() == num_observations;
  for (size_t idx = 0u; idx < num_observations; ++idx) {
    valid &=
        landmark.getAppearanceForObservationIndex(idx) == static_cast<int>(idx);
    valid &= distinct_appearances.count(static_cast<int>(idx)) > 0;
  }
  return valid;
}

TEST_F(LandmarkTest, TestSerializationNoAppearances) {
  const size_t num_observations = 200u;

  KeypointIdentifierList observations;
  generateObservations(num_observations, &observations);
  ASSERT_EQ(observations.size(), num_observations);

  landmark_.addObservations(observations);
  EXPECT_EQ(landmark_.numberOfObservations(), num_observations);
  EXPECT_DEATH(landmark_.getAppearances(), "");
  std::unordered_set<int> distinct_appearances;
  EXPECT_DEATH(landmark_.getAllDistinctAppearances(&distinct_appearances), "");

  vi_map::proto::Landmark proto_landmark;
  landmark_.serialize(&proto_landmark);

  EXPECT_EQ(
      proto_landmark.vertex_ids_size(), static_cast<int>(num_observations));
  EXPECT_EQ(
      proto_landmark.frame_indices_size(), static_cast<int>(num_observations));
  EXPECT_EQ(
      proto_landmark.keypoint_indices_size(),
      static_cast<int>(num_observations));
  EXPECT_EQ(proto_landmark.appearances_size(), 0);

  Landmark deserialized_landmark;
  deserialized_landmark.deserialize(proto_landmark);

  EXPECT_DEATH(landmark_.getAppearances(), "");
}

TEST_F(LandmarkTest, TestSerializationWithAppearances) {
  const size_t num_observations = 500u;

  KeypointIdentifierList observations;
  generateObservations(num_observations, &observations);
  ASSERT_EQ(observations.size(), num_observations);

  landmark_.addObservations(observations);
  EXPECT_EQ(landmark_.numberOfObservations(), num_observations);

  allocateIncrementalAppearances();

  EXPECT_TRUE(verifyIncrementalAppearances(landmark_));

  vi_map::proto::Landmark proto_landmark;
  landmark_.serialize(&proto_landmark);

  EXPECT_EQ(
      proto_landmark.vertex_ids_size(), static_cast<int>(num_observations));
  EXPECT_EQ(
      proto_landmark.frame_indices_size(), static_cast<int>(num_observations));
  EXPECT_EQ(
      proto_landmark.keypoint_indices_size(),
      static_cast<int>(num_observations));
  EXPECT_EQ(proto_landmark.appearances_size(), num_observations);

  Landmark deserialized_landmark;
  deserialized_landmark.deserialize(proto_landmark);

  EXPECT_TRUE(verifyIncrementalAppearances(deserialized_landmark));
}

TEST_F(LandmarkTest, TestClearObservationsAndAppearances) {
  const size_t num_observations = 500u;

  KeypointIdentifierList observations;
  generateObservations(num_observations, &observations);
  ASSERT_EQ(observations.size(), num_observations);

  landmark_.addObservations(observations);
  EXPECT_EQ(landmark_.numberOfObservations(), num_observations);

  landmark_.clearObservations();
  EXPECT_EQ(landmark_.numberOfObservations(), 0u);
  EXPECT_TRUE(landmark_.getObservations().empty());

  landmark_.addObservations(observations);
  allocateIncrementalAppearances();

  EXPECT_EQ(landmark_.getAppearances().size(), num_observations);

  landmark_.clearObservations();
  EXPECT_DEATH(landmark_.getAppearances(), "");
}

TEST_F(LandmarkTest, TestRemovalOfObservationsByIndexNoAppearances) {
  const size_t num_observations = 5u;

  KeypointIdentifierList observations;
  generateObservations(num_observations, &observations);
  ASSERT_EQ(observations.size(), num_observations);

  landmark_.addObservations(observations);
  EXPECT_EQ(landmark_.numberOfObservations(), num_observations);

  EXPECT_DEATH(landmark_.getAppearances(), "");
  std::unordered_set<int> distinct_appearances;
  EXPECT_DEATH(landmark_.getAllDistinctAppearances(&distinct_appearances), "");

  KeypointIdentifierList reduced_observations = observations;

  // Remove one by one using the index-based removal.
  reduced_observations.erase(reduced_observations.begin() + 1);
  landmark_.removeObservation(1u);
  EXPECT_EQ(landmark_.getObservations(), reduced_observations);

  reduced_observations.erase(reduced_observations.begin() + 3);
  landmark_.removeObservation(3u);
  EXPECT_EQ(landmark_.getObservations(), reduced_observations);

  reduced_observations.erase(reduced_observations.begin());
  landmark_.removeObservation(0u);
  EXPECT_EQ(landmark_.getObservations(), reduced_observations);

  reduced_observations.erase(reduced_observations.begin() + 1);
  landmark_.removeObservation(1u);
  EXPECT_EQ(landmark_.getObservations(), reduced_observations);

  reduced_observations.erase(reduced_observations.begin());
  landmark_.removeObservation(0u);
  EXPECT_TRUE(landmark_.getObservations().empty());
}

TEST_F(LandmarkTest, TestRemovalOfObservationsByObservationNoAppearances) {
  const size_t num_observations = 5u;

  KeypointIdentifierList observations;
  generateObservations(num_observations, &observations);
  ASSERT_EQ(observations.size(), num_observations);

  landmark_.addObservations(observations);
  EXPECT_EQ(landmark_.numberOfObservations(), num_observations);

  EXPECT_DEATH(landmark_.getAppearances(), "");
  std::unordered_set<int> distinct_appearances;
  EXPECT_DEATH(landmark_.getAllDistinctAppearances(&distinct_appearances), "");

  KeypointIdentifierList reduced_observations = observations;

  // Remove one by one using the KeypointIdentifier.
  landmark_.removeObservation(reduced_observations[2]);
  reduced_observations.erase(reduced_observations.begin() + 2);
  EXPECT_EQ(landmark_.getObservations(), reduced_observations);

  landmark_.removeObservation(reduced_observations[2]);
  reduced_observations.erase(reduced_observations.begin() + 2);
  EXPECT_EQ(landmark_.getObservations(), reduced_observations);

  landmark_.removeObservation(reduced_observations[0]);
  reduced_observations.erase(reduced_observations.begin());
  EXPECT_EQ(landmark_.getObservations(), reduced_observations);

  landmark_.removeObservation(reduced_observations[1]);
  reduced_observations.erase(reduced_observations.begin() + 1);
  EXPECT_EQ(landmark_.getObservations(), reduced_observations);

  landmark_.removeObservation(reduced_observations[0]);
  reduced_observations.erase(reduced_observations.begin());
  EXPECT_TRUE(landmark_.getObservations().empty());
}

TEST_F(LandmarkTest, TestRemovalOfObservationsByVertexNoAppearances) {
  const size_t num_observations = 5u;

  KeypointIdentifierList observations;
  generateObservations(num_observations, &observations);
  ASSERT_EQ(observations.size(), num_observations);

  // Set vertex of second and fifth equal.
  observations[4].frame_id.vertex_id = observations[1].frame_id.vertex_id;

  // Set vertex of first and third equal.
  observations[2].frame_id.vertex_id = observations[0].frame_id.vertex_id;

  landmark_.addObservations(observations);
  EXPECT_EQ(landmark_.numberOfObservations(), num_observations);

  EXPECT_DEATH(landmark_.getAppearances(), "");
  std::unordered_set<int> distinct_appearances;
  EXPECT_DEATH(landmark_.getAllDistinctAppearances(&distinct_appearances), "");

  KeypointIdentifierList reduced_observations = observations;

  // Observations: {0, 1, 2, 3, 4}.
  // Vertices:     {a, b, a, d, b}.

  // Remove by vertex.
  landmark_.removeAllObservationsOfVertex(
      reduced_observations[2].frame_id.vertex_id);
  // Observations: {1, 3, 4}.
  // Vertices:     {b, d, b}.
  reduced_observations.erase(reduced_observations.begin() + 2);
  reduced_observations.erase(reduced_observations.begin());
  EXPECT_EQ(landmark_.getObservations(), reduced_observations);

  landmark_.removeAllObservationsOfVertex(
      reduced_observations[1].frame_id.vertex_id);
  // Observations: {1, 4}.
  // Vertices:     {b, b}.
  reduced_observations.erase(reduced_observations.begin() + 1);
  EXPECT_EQ(landmark_.getObservations(), reduced_observations);

  landmark_.removeAllObservationsOfVertex(
      reduced_observations[1].frame_id.vertex_id);
  // Observations: {}.
  // Vertices:     {}.
  EXPECT_TRUE(landmark_.getObservations().empty());
}

TEST_F(
    LandmarkTest, TestRemovalOfObservationsByVertexAndFrameIndexNoAppearances) {
  const size_t num_observations = 5u;

  KeypointIdentifierList observations;
  generateObservations(num_observations, &observations);
  ASSERT_EQ(observations.size(), num_observations);

  // Set vertex of second and fifth equal.
  observations[4].frame_id.vertex_id = observations[1].frame_id.vertex_id;

  // Set vertex of first and third equal.
  observations[2].frame_id.vertex_id = observations[0].frame_id.vertex_id;

  landmark_.addObservations(observations);
  EXPECT_EQ(landmark_.numberOfObservations(), num_observations);

  EXPECT_DEATH(landmark_.getAppearances(), "");
  std::unordered_set<int> distinct_appearances;
  EXPECT_DEATH(landmark_.getAllDistinctAppearances(&distinct_appearances), "");

  KeypointIdentifierList reduced_observations = observations;

  // Observations: {0, 1, 2, 3, 4}.
  // Vertices:     {a, b, a, d, b}.
  // Frame Index:  {0, 1, 0, 1, 0}.

  // Remove by vertex.
  landmark_.removeAllObservationsOfVertexAndFrame(
      reduced_observations[2].frame_id.vertex_id, 0u);
  // Observations: {1, 3, 4}.
  // Vertices:     {b, d, b}.
  // Frame Index:  {1, 1, 0}.
  reduced_observations.erase(reduced_observations.begin() + 2);
  reduced_observations.erase(reduced_observations.begin());
  EXPECT_EQ(landmark_.getObservations(), reduced_observations);

  landmark_.removeAllObservationsOfVertexAndFrame(
      reduced_observations[1].frame_id.vertex_id,
      0u);  // (Nothing should happen.)
  // Observations: {1, 3, 4}.
  // Vertices:     {b, d, b}.
  // Frame Index:  {1, 1, 0}.
  EXPECT_EQ(landmark_.getObservations(), reduced_observations);

  landmark_.removeAllObservationsOfVertexAndFrame(
      reduced_observations[1].frame_id.vertex_id, 1u);
  // Observations: {1, 4}.
  // Vertices:     {b, b}.
  // Frame Index:  {1, 0}.
  reduced_observations.erase(reduced_observations.begin() + 1);
  EXPECT_EQ(landmark_.getObservations(), reduced_observations);

  landmark_.removeAllObservationsOfVertexAndFrame(
      reduced_observations[1].frame_id.vertex_id, 0u);
  // Observations: {1}.
  // Vertices:     {b}.
  // Frame Index:  {1}.
  reduced_observations.erase(reduced_observations.begin() + 1);
  EXPECT_EQ(landmark_.getObservations(), reduced_observations);

  landmark_.removeAllObservationsOfVertexAndFrame(
      reduced_observations[0].frame_id.vertex_id, 1u);
  // Observations: {}.
  // Vertices:     {}.
  // Frame Index:  {}.
  EXPECT_TRUE(landmark_.getObservations().empty());
}

TEST_F(LandmarkTest, TestRemovalOfObservationsByIndexWithAppearances) {
  const size_t num_observations = 5u;

  KeypointIdentifierList observations;
  generateObservations(num_observations, &observations);
  ASSERT_EQ(observations.size(), num_observations);

  landmark_.addObservations(observations);
  EXPECT_EQ(landmark_.numberOfObservations(), num_observations);

  allocateIncrementalAppearances();

  std::unordered_set<int> distinct_appearances;
  landmark_.getAllDistinctAppearances(&distinct_appearances);
  EXPECT_EQ(distinct_appearances.size(), num_observations);

  std::vector<int> reduced_appearances = {0, 1, 2, 3, 4};
  EXPECT_EQ(landmark_.getAppearances(), reduced_appearances);

  // Remove one by one using the index-based removal.
  landmark_.removeObservation(1u);
  reduced_appearances.erase(reduced_appearances.begin() + 1);
  EXPECT_EQ(landmark_.getAppearances(), reduced_appearances);

  landmark_.removeObservation(3u);
  reduced_appearances.erase(reduced_appearances.begin() + 3);
  EXPECT_EQ(landmark_.getAppearances(), reduced_appearances);

  landmark_.removeObservation(0u);
  reduced_appearances.erase(reduced_appearances.begin());
  EXPECT_EQ(landmark_.getAppearances(), reduced_appearances);

  landmark_.removeObservation(1u);
  reduced_appearances.erase(reduced_appearances.begin() + 1);
  EXPECT_EQ(landmark_.getAppearances(), reduced_appearances);

  landmark_.removeObservation(0u);
  reduced_appearances.erase(reduced_appearances.begin());
  EXPECT_DEATH(landmark_.getAppearances(), "");
}

TEST_F(LandmarkTest, TestRemovalOfObservationsByObservationWithAppearances) {
  const size_t num_observations = 5u;

  KeypointIdentifierList observations;
  generateObservations(num_observations, &observations);
  ASSERT_EQ(observations.size(), num_observations);

  landmark_.addObservations(observations);
  EXPECT_EQ(landmark_.numberOfObservations(), num_observations);

  allocateIncrementalAppearances();

  std::unordered_set<int> distinct_appearances;
  landmark_.getAllDistinctAppearances(&distinct_appearances);
  EXPECT_EQ(distinct_appearances.size(), num_observations);

  std::vector<int> reduced_appearances = {0, 1, 2, 3, 4};
  EXPECT_EQ(landmark_.getAppearances(), reduced_appearances);

  // Remove one by one using the KeypointIdentifier.
  landmark_.removeObservation(observations[2]);
  reduced_appearances.erase(reduced_appearances.begin() + 2);
  EXPECT_EQ(landmark_.getAppearances(), reduced_appearances);
  // Apperances left: {0,1,3,4}.

  landmark_.removeObservation(observations[3]);
  reduced_appearances.erase(reduced_appearances.begin() + 2);
  EXPECT_EQ(landmark_.getAppearances(), reduced_appearances);
  // Apperances left: {0,1,4}.

  landmark_.removeObservation(observations[0]);
  reduced_appearances.erase(reduced_appearances.begin());
  EXPECT_EQ(landmark_.getAppearances(), reduced_appearances);
  // Apperances left: {1,4}.

  landmark_.removeObservation(observations[4]);
  reduced_appearances.erase(reduced_appearances.begin() + 1);
  EXPECT_EQ(landmark_.getAppearances(), reduced_appearances);
  // Apperances left: {1}.

  landmark_.removeObservation(observations[1]);
  reduced_appearances.erase(reduced_appearances.begin());
  EXPECT_DEATH(landmark_.getAppearances(), "");
}

TEST_F(LandmarkTest, TestRemovalOfObservationsByVertexWithAppearances) {
  const size_t num_observations = 5u;

  KeypointIdentifierList observations;
  generateObservations(num_observations, &observations);
  ASSERT_EQ(observations.size(), num_observations);

  // Set vertex of second and fifth equal.
  observations[4].frame_id.vertex_id = observations[1].frame_id.vertex_id;

  // Set vertex of first and third equal.
  observations[2].frame_id.vertex_id = observations[0].frame_id.vertex_id;

  landmark_.addObservations(observations);
  EXPECT_EQ(landmark_.numberOfObservations(), num_observations);

  allocateIncrementalAppearances();

  std::unordered_set<int> distinct_appearances;
  landmark_.getAllDistinctAppearances(&distinct_appearances);
  EXPECT_EQ(distinct_appearances.size(), num_observations);

  std::vector<int> reduced_appearances = {0, 1, 2, 3, 4};
  // Observations: {0, 1, 2, 3, 4}.
  // Vertices:     {a, b, a, d, b}.

  // Remove by vertex.
  landmark_.removeAllObservationsOfVertex(observations[2].frame_id.vertex_id);
  // Observations: {1, 3, 4}.
  // Vertices:     {b, d, b}.
  reduced_appearances.erase(reduced_appearances.begin() + 2);
  reduced_appearances.erase(reduced_appearances.begin());
  EXPECT_EQ(landmark_.getAppearances(), reduced_appearances);

  landmark_.removeAllObservationsOfVertex(observations[3].frame_id.vertex_id);
  // Observations: {1, 4}.
  // Vertices:     {b, b}.
  reduced_appearances.erase(reduced_appearances.begin() + 1);
  EXPECT_EQ(landmark_.getAppearances(), reduced_appearances);

  landmark_.removeAllObservationsOfVertex(observations[4].frame_id.vertex_id);
  // Observations: {}.
  // Vertices:     {}.
  EXPECT_DEATH(landmark_.getAppearances(), "");
}

TEST_F(
    LandmarkTest,
    TestRemovalOfObservationsByVertexAndFrameIndexWithAppearances) {
  const size_t num_observations = 5u;

  KeypointIdentifierList observations;
  generateObservations(num_observations, &observations);
  ASSERT_EQ(observations.size(), num_observations);

  // Set vertex of second and fifth equal.
  observations[4].frame_id.vertex_id = observations[1].frame_id.vertex_id;

  // Set vertex of first and third equal.
  observations[2].frame_id.vertex_id = observations[0].frame_id.vertex_id;

  landmark_.addObservations(observations);
  EXPECT_EQ(landmark_.numberOfObservations(), num_observations);

  allocateIncrementalAppearances();

  std::unordered_set<int> distinct_appearances;
  landmark_.getAllDistinctAppearances(&distinct_appearances);
  EXPECT_EQ(distinct_appearances.size(), num_observations);

  std::vector<int> reduced_appearances = {0, 1, 2, 3, 4};

  // Observations: {0, 1, 2, 3, 4}.
  // Vertices:     {a, b, a, d, b}.
  // Frame Index:  {0, 1, 0, 1, 0}.

  // Remove by vertex.
  landmark_.removeAllObservationsOfVertexAndFrame(
      observations[2].frame_id.vertex_id, 0u);
  // Observations: {1, 3, 4}.
  // Vertices:     {b, d, b}.
  // Frame Index:  {1, 1, 0}.
  reduced_appearances.erase(reduced_appearances.begin() + 2);
  reduced_appearances.erase(reduced_appearances.begin());
  EXPECT_EQ(landmark_.getAppearances(), reduced_appearances);

  landmark_.removeAllObservationsOfVertexAndFrame(
      observations[3].frame_id.vertex_id, 0u);  // (Nothing should happen.)
  // Observations: {1, 3, 4}.
  // Vertices:     {b, d, b}.
  // Frame Index:  {1, 1, 0}.
  EXPECT_EQ(landmark_.getAppearances(), reduced_appearances);

  landmark_.removeAllObservationsOfVertexAndFrame(
      observations[3].frame_id.vertex_id, 1u);
  // Observations: {1, 4}.
  // Vertices:     {b, b}.
  // Frame Index:  {1, 0}.
  reduced_appearances.erase(reduced_appearances.begin() + 1);
  EXPECT_EQ(landmark_.getAppearances(), reduced_appearances);

  landmark_.removeAllObservationsOfVertexAndFrame(
      observations[4].frame_id.vertex_id, 0u);
  // Observations: {1}.
  // Vertices:     {b}.
  // Frame Index:  {1}.
  reduced_appearances.erase(reduced_appearances.begin() + 1);
  EXPECT_EQ(landmark_.getAppearances(), reduced_appearances);

  landmark_.removeAllObservationsOfVertexAndFrame(
      observations[1].frame_id.vertex_id, 1u);
  // Observations: {}.
  // Vertices:     {}.
  // Frame Index:  {}.
  EXPECT_DEATH(landmark_.getAppearances(), "");
}
}  // namespace vi_map

MAPLAB_UNITTEST_ENTRYPOINT
