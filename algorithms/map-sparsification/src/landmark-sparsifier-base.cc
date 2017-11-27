#include "map-sparsification/landmark-sparsifier-base.h"

#include <aslam/common/descriptor-utils.h>
#include <limits>

namespace map_sparsification {

void LandmarkSparsifierBase::sparsifyLandmarks() {
  CHECK_NOTNULL(map_);

  LOG(INFO) << "Store landmarks number before sparsification: "
            << map_->numLandmarks();

  std::unordered_set<vi_map::LandmarkId> landmark_ids;
  getSetOfDeletableLandmarks(&landmark_ids);

  CHECK_GT(landmark_ids.size(), num_of_landmarks_to_remove_)
      << "There are not enough landmarks to remove the desired landmark number"
      << " in sparsification process.";

  sparsifyLandmarksImpl(&landmark_ids);

  LOG(INFO) << "Store landmarks number after sparsification: "
            << map_->numLandmarks();
}

bool LandmarkSparsifierBase::shouldLandmarkBeKept(
    const vi_map::LandmarkId& landmark_id) const {
  if (!missions_to_skip_.empty()) {
    std::unordered_set<vi_map::MissionId> missions;
    map_->getLandmarkObserverMissions(landmark_id, &missions);

    for (const vi_map::MissionId& mission_id_to_skip :
         missions_to_skip_) {
      if (missions.count(mission_id_to_skip) > 0) {
        return true;
      }
    }
  }
  return false;
}

bool LandmarkSparsifierBase::removeLandmarkIfMinPointsPerFrameSatisfied(
    const vi_map::LandmarkId& landmark_id,
    int min_frame_keypoints) {
  bool can_be_removed = true;

  // Verify if we won't "break" any of the observer keyframes.
  const vi_map::Landmark& landmark = map_->getLandmark(landmark_id);
  for (const vi_map::KeypointIdentifier observation :
       landmark.getObservations()) {
    const vi_map::Vertex& vertex =
        map_->getVertex(observation.frame_id.vertex_id);
    if (vertex.numValidObservedLandmarkIdsInAllFrames() < min_frame_keypoints) {
      can_be_removed = false;
      break;
    }
  }

  if (can_be_removed) {
    map_->removeLandmark(landmark_id);
  }

  return can_be_removed;
}

bool LandmarkSparsifierBase::removeLandmarkIfTooDistant(
    const vi_map::LandmarkId& landmark_id,
    double threshold_meters) {
  const vi_map::Landmark& landmark = map_->getLandmark(landmark_id);

  // Our assumptions do not allow a store landmark to have zero observers.
  CHECK(landmark.hasObservations());

  double min_distance_from_observer = std::numeric_limits<double>::max();
  for (const vi_map::KeypointIdentifier observation :
       landmark.getObservations()) {
    const vi_map::Vertex& vertex =
        map_->getVertex(observation.frame_id.vertex_id);
    Eigen::Vector3d I_p_fi = map_->getLandmark_p_I_fi(landmark_id, vertex);

    double I_p_fi_norm = I_p_fi.norm();
    if (I_p_fi_norm < min_distance_from_observer) {
      min_distance_from_observer = I_p_fi_norm;
    }
  }

  if (min_distance_from_observer > threshold_meters) {
    map_->removeLandmark(landmark_id);
    return true;
  }
  return false;
}

bool LandmarkSparsifierBase::removeLandmarkIfDescriptorStdDevTooLarge(
    const vi_map::LandmarkId& landmark_id,
    double std_dev_threshold) {
  vi_map::VIMap::DescriptorsType descriptors;
  map_->getLandmarkDescriptors(landmark_id, &descriptors);
  if (aslam::common::descriptor_utils::descriptorMeanStandardDeviation(
          descriptors) > std_dev_threshold) {
    map_->removeLandmark(landmark_id);
    return true;
  }
  return false;
}

void LandmarkSparsifierBase::getSetOfDeletableLandmarks(
    std::unordered_set<vi_map::LandmarkId>* landmark_ids) const {
  CHECK_NOTNULL(landmark_ids);

  landmark_ids->clear();
  map_->getAllLandmarkIds(landmark_ids);
  std::unordered_set<vi_map::LandmarkId> deletable_landmark_ids =
      *landmark_ids;

  for (const vi_map::LandmarkId& store_landmark_id : *landmark_ids) {
    if (shouldLandmarkBeKept(store_landmark_id)) {
      deletable_landmark_ids.erase(store_landmark_id);
    }
  }
  landmark_ids->swap(deletable_landmark_ids);
}

}  // namespace map_sparsification
