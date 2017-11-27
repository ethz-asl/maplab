#include <memory>
#include <string>

#include <Eigen/Core>
#include <aslam/common/hash-id.h>
#include <maplab-common/pose_types.h>
#include <maplab-common/test/testing-entrypoint.h>
#include <maplab-common/test/testing-predicates.h>
#include <maplab-common/unique-id.h>
#include <posegraph/unique-id.h>

#include <vi-map/unique-id.h>
#include <vi-map/vi_map.pb.h>

#include "localization-summary-map/localization-summary-map.h"
#include "localization-summary-map/localization-summary-map.pb.h"

namespace summary_map {

class LocalizationSummaryMapTest : public ::testing::Test {
 protected:
  virtual void SetUp() {}

  void constructLocalizationSummaryMap();
  void serializeAndDeserialize();

  summary_map::LocalizationSummaryMap::Ptr initial_summary_map_;
  summary_map::LocalizationSummaryMap::Ptr summary_map_from_msg_;
};

void LocalizationSummaryMapTest::serializeAndDeserialize() {
  proto::LocalizationSummaryMap summary_map_proto, proto_from_msg;
  CHECK(initial_summary_map_);
  initial_summary_map_->serialize(&summary_map_proto);
  std::string serialized_summary_map = summary_map_proto.SerializeAsString();

  proto_from_msg.ParseFromString(serialized_summary_map);

  summary_map_from_msg_.reset(new summary_map::LocalizationSummaryMap());
  summary_map_from_msg_->deserialize(
      initial_summary_map_->id(), proto_from_msg);
}

void LocalizationSummaryMapTest::constructLocalizationSummaryMap() {
  initial_summary_map_.reset(new summary_map::LocalizationSummaryMap());

  summary_map::LocalizationSummaryMapId id;
  common::generateId(&id);
  initial_summary_map_->setId(id);

  constexpr int kNumLandmarks = 50;
  Eigen::Matrix3Xd G_landmark_position;
  G_landmark_position.resize(Eigen::NoChange, kNumLandmarks);
  G_landmark_position.setRandom();
  initial_summary_map_->setGLandmarkPosition(G_landmark_position);

  constexpr int kNumObservations = 120;
  constexpr int kNumDescriptorDimensions = 10;
  Eigen::MatrixXf descriptors;
  descriptors.resize(kNumDescriptorDimensions, kNumObservations);
  descriptors.setRandom();
  initial_summary_map_->setProjectedDescriptors(descriptors);

  constexpr int kNumKeyframes = 10;
  Eigen::Matrix<unsigned int, Eigen::Dynamic, 1> observer_indices;
  observer_indices.resize(kNumObservations, Eigen::NoChange);
  for (int i = 0; i < kNumObservations; ++i) {
    observer_indices(i, 0) = i % kNumKeyframes;
  }
  initial_summary_map_->setObserverIndices(observer_indices);

  Eigen::Matrix3Xd observer_positions;
  observer_positions.setRandom(3, observer_indices.cols());
  initial_summary_map_->setGObserverPosition(observer_positions);

  Eigen::Matrix<unsigned int, Eigen::Dynamic, 1> observation_to_landmark_index;
  observation_to_landmark_index.resize(kNumObservations, Eigen::NoChange);
  for (int i = 0; i < kNumObservations; ++i) {
    observation_to_landmark_index(i, 0) = i % kNumLandmarks;
  }
  initial_summary_map_->setObservationToLandmarkIndex(
      observation_to_landmark_index);
}

TEST_F(
    LocalizationSummaryMapTest,
    LocalizationSummaryMapSerializeAndDeserializeTest) {
  constructLocalizationSummaryMap();
  serializeAndDeserialize();

  EXPECT_EQ(initial_summary_map_->id(), summary_map_from_msg_->id());
  EXPECT_NEAR_EIGEN(
      initial_summary_map_->GLandmarkPosition(),
      summary_map_from_msg_->GLandmarkPosition(), 1e-10);
  EXPECT_NEAR_EIGEN(
      initial_summary_map_->GObserverPosition(),
      summary_map_from_msg_->GObserverPosition(), 1e-10);
  EXPECT_NEAR_EIGEN(
      initial_summary_map_->projectedDescriptors(),
      summary_map_from_msg_->projectedDescriptors(), 1e-10);
  EXPECT_TRUE(
      initial_summary_map_->observerIndices() ==
      summary_map_from_msg_->observerIndices());
  EXPECT_TRUE(
      initial_summary_map_->observationToLandmarkIndex() ==
      summary_map_from_msg_->observationToLandmarkIndex());

  EXPECT_EQ(*initial_summary_map_, *summary_map_from_msg_);
}

TEST_F(
    LocalizationSummaryMapTest,
    LocalizationSummaryMapSerializeAndDeserializeNotEqualTest) {
  constructLocalizationSummaryMap();
  serializeAndDeserialize();

  constexpr int kNumLandmarks = 50;
  Eigen::Matrix3Xd G_landmark_position;
  G_landmark_position.resize(Eigen::NoChange, kNumLandmarks);
  G_landmark_position.setRandom();
  initial_summary_map_->setGLandmarkPosition(G_landmark_position);

  EXPECT_NE(*initial_summary_map_, *summary_map_from_msg_);
}

}  // namespace summary_map

MAPLAB_UNITTEST_ENTRYPOINT
