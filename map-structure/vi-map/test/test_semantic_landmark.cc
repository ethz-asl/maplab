#include <vector>

#include <aslam/common/memory.h>
#include <glog/logging.h>
#include <gtest/gtest.h>
#include <maplab-common/test/testing-entrypoint.h>
#include <maplab-common/unique-id.h>
#include <posegraph/unique-id.h>

#include "vi-map/semantic-landmark.h"
#include "vi-map/unique-id.h"
#include "vi-map/vi_map.pb.h"

namespace vi_map {

class SemanticLandmarkTest : public ::testing::Test {
 protected:
  virtual void SetUp() {}

  void generateObservations(
      size_t num_observations_to_add, SemanticObjectIdentifierList* observations) {
    CHECK_NOTNULL(observations)->clear();

    for (size_t idx = 0u; idx < num_observations_to_add; ++idx) {
      SemanticObjectIdentifier observation;
      common::generateId(&observation.frame_id.vertex_id);
      observation.frame_id.frame_index = idx % 2u;
      observation.measurement_index = idx;

      observations->emplace_back(observation);
    }
  }

  SemanticLandmark landmark_;
};

TEST_F(SemanticLandmarkTest, TestSerialization) {
  const size_t num_observations = 200u;

  SemanticObjectIdentifierList observations;
  generateObservations(num_observations, &observations);
  ASSERT_EQ(observations.size(), num_observations);

  landmark_.addObservations(observations);
  EXPECT_EQ(landmark_.numberOfObservations(), num_observations);

  vi_map::proto::SemanticLandmark proto_landmark;
  landmark_.serialize(&proto_landmark);

  EXPECT_EQ(
      proto_landmark.vertex_ids_size(), static_cast<int>(num_observations));
  EXPECT_EQ(
      proto_landmark.frame_indices_size(), static_cast<int>(num_observations));
  EXPECT_EQ(
      proto_landmark.measurement_indices_size(),
      static_cast<int>(num_observations));

  SemanticLandmark deserialized_landmark;
  deserialized_landmark.deserialize(proto_landmark);
  EXPECT_TRUE(landmark_ == deserialized_landmark);
}


TEST_F(SemanticLandmarkTest, TestClearObservations) {
  const size_t num_observations = 500u;

  SemanticObjectIdentifierList observations;
  generateObservations(num_observations, &observations);
  ASSERT_EQ(observations.size(), num_observations);

  landmark_.addObservations(observations);
  EXPECT_EQ(landmark_.numberOfObservations(), num_observations);

  landmark_.clearObservations();
  EXPECT_EQ(landmark_.numberOfObservations(), 0u);
  EXPECT_TRUE(landmark_.getObservations().empty());
}

TEST_F(SemanticLandmarkTest, TestRemovalOfObservationsByIndex) {
  const size_t num_observations = 5u;

  SemanticObjectIdentifierList observations;
  generateObservations(num_observations, &observations);
  ASSERT_EQ(observations.size(), num_observations);

  landmark_.addObservations(observations);
  EXPECT_EQ(landmark_.numberOfObservations(), num_observations);

  SemanticObjectIdentifierList reduced_observations = observations;

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

TEST_F(SemanticLandmarkTest, TestRemovalOfObservationsByObservation) {
  const size_t num_observations = 5u;

  SemanticObjectIdentifierList observations;
  generateObservations(num_observations, &observations);
  ASSERT_EQ(observations.size(), num_observations);

  landmark_.addObservations(observations);
  EXPECT_EQ(landmark_.numberOfObservations(), num_observations);

  SemanticObjectIdentifierList reduced_observations = observations;

  // Remove one by one using the SemanticObjectIdentifier.
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

TEST_F(SemanticLandmarkTest, TestRemovalOfObservationsByVertex) {
  const size_t num_observations = 5u;

  SemanticObjectIdentifierList observations;
  generateObservations(num_observations, &observations);
  ASSERT_EQ(observations.size(), num_observations);

  // Set vertex of second and fifth equal.
  observations[4].frame_id.vertex_id = observations[1].frame_id.vertex_id;

  // Set vertex of first and third equal.
  observations[2].frame_id.vertex_id = observations[0].frame_id.vertex_id;

  landmark_.addObservations(observations);
  EXPECT_EQ(landmark_.numberOfObservations(), num_observations);

  SemanticObjectIdentifierList reduced_observations = observations;

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
    SemanticLandmarkTest, TestRemovalOfObservationsByVertexAndFrameIndex) {
  const size_t num_observations = 5u;

  SemanticObjectIdentifierList observations;
  generateObservations(num_observations, &observations);
  ASSERT_EQ(observations.size(), num_observations);

  // Set vertex of second and fifth equal.
  observations[4].frame_id.vertex_id = observations[1].frame_id.vertex_id;

  // Set vertex of first and third equal.
  observations[2].frame_id.vertex_id = observations[0].frame_id.vertex_id;

  landmark_.addObservations(observations);
  EXPECT_EQ(landmark_.numberOfObservations(), num_observations);

  SemanticObjectIdentifierList reduced_observations = observations;

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
}  // namespace vi_map

MAPLAB_UNITTEST_ENTRYPOINT
