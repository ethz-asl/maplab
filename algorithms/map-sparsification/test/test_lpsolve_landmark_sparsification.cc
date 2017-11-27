#include <gtest/gtest.h>
#include <maplab-common/test/testing-entrypoint.h>
#include <maplab-common/test/testing-predicates.h>
#include <maplab-common/vector-window-operations.h>
#include <vi-map-helpers/vi-map-partitioner.h>
#include <vi-mapping-test-app/vi-mapping-test-app.h>

#include "map-sparsification/graph-partition-sampler.h"
#include "map-sparsification/sampler-factory.h"

namespace map_sparsification {

class ViMappingTest : public ::testing::Test {
 protected:
  virtual void SetUp() {
    test_app_.loadDataset("./test_maps/vi_app_test");

    // Set quality of all landmarks to good as they are unknown in the test
    // map. We save time of retriangulation.
    vi_map::VIMap* vi_map = CHECK_NOTNULL(test_app_.getMapMutable());
    vi_map::LandmarkIdList landmark_ids;
    vi_map->getAllLandmarkIds(&landmark_ids);
    for (const vi_map::LandmarkId& landmark_id : landmark_ids) {
      vi_map->getLandmark(landmark_id)
          .setQuality(vi_map::Landmark::Quality::kGood);
    }
  }

  void constructSampler() {
    sampler_ = createSampler(SamplerBase::Type::kLpsolveIlp);
  }

  void sampleLandmarks(vi_map::LandmarkIdSet* landmarks_to_keep) {
    CHECK(sampler_ != nullptr);
    CHECK_NOTNULL(landmarks_to_keep);

    const vi_map::VIMap& vi_map = *CHECK_NOTNULL(test_app_.getMapMutable());

    const size_t num_landmarks = vi_map.numLandmarksInIndex();
    const size_t desired_num_landmarks = 0.25 * num_landmarks;
    sampler_->sample(vi_map, desired_num_landmarks, landmarks_to_keep);

    EXPECT_EQ(desired_num_landmarks, landmarks_to_keep->size());
  }

  void sampleLandmarksWithPartitioning(
      vi_map::LandmarkIdSet* landmarks_to_keep) {
    CHECK(sampler_ != nullptr);
    CHECK_NOTNULL(landmarks_to_keep);
    const vi_map::VIMap& vi_map = *CHECK_NOTNULL(test_app_.getMapMutable());

    GraphPartitionSampler partition_sampler(sampler_);

    const size_t num_landmarks = vi_map.numLandmarksInIndex();
    const size_t desired_num_landmarks = 0.25 * num_landmarks;
    partition_sampler.sample(vi_map, desired_num_landmarks, landmarks_to_keep);

    EXPECT_LE(landmarks_to_keep->size(), desired_num_landmarks);
  }

  void evaluteLandmarkSelection(
      const vi_map::LandmarkIdSet& landmarks_to_keep) {
    const vi_map::VIMap& vi_map = *CHECK_NOTNULL(test_app_.getMapMutable());
    vi_map::LandmarkIdList landmark_ids;
    vi_map.getAllLandmarkIds(&landmark_ids);

    // This function verifies if kept landmarks have on average more
    // observations that the landmarks that are deemed safe to remove.
    std::vector<size_t> nums_observations_kept_landmarks;
    std::vector<size_t> nums_observations_removed_landmarks;

    nums_observations_kept_landmarks.reserve(landmarks_to_keep.size());
    nums_observations_removed_landmarks.reserve(
        landmark_ids.size() - landmarks_to_keep.size());

    for (const vi_map::LandmarkId& landmark_id : landmark_ids) {
      const size_t num_observations =
          vi_map.getLandmark(landmark_id).numberOfObservations();
      if (landmarks_to_keep.count(landmark_id) > 0u) {
        nums_observations_kept_landmarks.push_back(num_observations);
      } else {
        nums_observations_removed_landmarks.push_back(num_observations);
      }
    }

    CHECK_EQ(
        nums_observations_kept_landmarks.size() +
            nums_observations_removed_landmarks.size(),
        landmark_ids.size());

    constexpr size_t kInvalidValue = -1;
    const size_t avg_observers_kept = common::window_vec_ops::computeAverage(
        nums_observations_kept_landmarks, kInvalidValue);
    const size_t avg_observers_removed = common::window_vec_ops::computeAverage(
        nums_observations_removed_landmarks, kInvalidValue);

    // Make sure the difference between kept and removed landmarks is
    // reasonably large.
    EXPECT_GT(avg_observers_kept, avg_observers_removed + 10);
  }

 private:
  visual_inertial_mapping::VIMappingTestApp test_app_;
  SamplerBase::Ptr sampler_;
};

TEST_F(ViMappingTest, LpsolveLandmarkSparsificationWorks) {
  constructSampler();

  vi_map::LandmarkIdSet landmarks_to_keep;
  sampleLandmarks(&landmarks_to_keep);

  evaluteLandmarkSelection(landmarks_to_keep);
}

TEST_F(ViMappingTest, PartitionedLpsolveLandmarkSparsificationWorks) {
  constructSampler();

  vi_map::LandmarkIdSet landmarks_to_keep;
  sampleLandmarksWithPartitioning(&landmarks_to_keep);

  evaluteLandmarkSelection(landmarks_to_keep);
}

}  // namespace map_sparsification

MAPLAB_UNITTEST_ENTRYPOINT
