#ifndef MAP_SPARSIFICATION_LANDMARK_SPARSIFIER_BASE_H_
#define MAP_SPARSIFICATION_LANDMARK_SPARSIFIER_BASE_H_

#include <string>
#include <unordered_set>

#include <glog/logging.h>

#include <maplab-common/macros.h>
#include <vi-map/vi-map.h>

namespace map_sparsification {

class LandmarkSparsifierBase {
 public:
  MAPLAB_POINTER_TYPEDEFS(LandmarkSparsifierBase);

  virtual ~LandmarkSparsifierBase() {}

  void setMap(vi_map::VIMap* map) { map_ = CHECK_NOTNULL(map); }

  void setNumberOfLandmarksToRemove(unsigned int num_of_landmarks_to_remove) {
    LOG(INFO) << "Number of landmarks to be removed: "
              << num_of_landmarks_to_remove;
    num_of_landmarks_to_remove_ = num_of_landmarks_to_remove;
  }

  void setMissionsToSkip(const vi_map::MissionIdSet& missions_to_skip) {
    missions_to_skip_ = missions_to_skip;
  }

  void sparsifyLandmarks();

  virtual const std::string& info() const = 0;

 protected:
  bool shouldLandmarkBeKept(
      const vi_map::LandmarkId& landmark_id) const;

  bool removeLandmarkIfMinPointsPerFrameSatisfied(
      const vi_map::LandmarkId& landmark_id,
      int min_frame_keypoints);

  bool removeLandmarkIfTooDistant(
      const vi_map::LandmarkId& landmark_id,
      double threshold_meters);

  bool removeLandmarkIfDescriptorStdDevTooLarge(
      const vi_map::LandmarkId& landmark_id,
      double LandmarkId);

  void getSetOfDeletableLandmarks(
      vi_map::LandmarkIdSet* landmark_ids) const;

  virtual void sparsifyLandmarksImpl(
      vi_map::LandmarkIdSet* landmark_ids) = 0;

  vi_map::VIMap* map_;
  unsigned int num_of_landmarks_to_remove_;
  vi_map::MissionIdSet missions_to_skip_;
};

}  // namespace map_sparsification

#endif  // MAP_SPARSIFICATION_LANDMARK_SPARSIFIER_BASE_H_
