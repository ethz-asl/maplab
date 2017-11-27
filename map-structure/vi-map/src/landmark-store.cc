#include <vi-map/landmark-store.h>

#include <glog/logging.h>

namespace vi_map {

double* LandmarkStore::get_p_B_Mutable(const LandmarkId& landmark_id) {
  LandmarkIdToIdxMap::const_iterator it;
  it = landmark_id_map_.find(landmark_id);
  CHECK(it != landmark_id_map_.end());

  return landmarks_[it->second].get_p_B_Mutable();
}

Landmark& LandmarkStore::getLandmark(const LandmarkId& landmark_id) {
  LandmarkIdToIdxMap::const_iterator it;
  it = landmark_id_map_.find(landmark_id);
  CHECK(it != landmark_id_map_.end());

  return landmarks_[it->second];
}

const Landmark& LandmarkStore::getLandmark(
    const LandmarkId& landmark_id) const {
  LandmarkIdToIdxMap::const_iterator it;
  it = landmark_id_map_.find(landmark_id);
  CHECK(it != landmark_id_map_.end());
  CHECK_LT(static_cast<unsigned int>(it->second), landmarks_.size());
  return landmarks_[it->second];
}

void LandmarkStore::addLandmark(const Landmark& landmark) {
  const unsigned int idx = landmarks_.size();
  landmarks_.push_back(landmark);
  landmark_id_map_.insert(std::make_pair(landmark.id(), idx));
  CHECK_EQ(landmarks_.size(), landmark_id_map_.size());
}

bool LandmarkStore::hasLandmark(const LandmarkId& landmark_id) const {
  return landmark_id_map_.count(landmark_id);
}

unsigned int LandmarkStore::size() const {
  return landmarks_.size();
}

void LandmarkStore::removeLandmark(const LandmarkId& landmark_id) {
  LandmarkIdToIdxMap::const_iterator it;
  it = landmark_id_map_.find(landmark_id);
  CHECK(it != landmark_id_map_.end());
  const unsigned int erased_landmark_index = it->second;
  landmark_id_map_.erase(landmark_id);
  landmarks_.erase(landmarks_.begin() + erased_landmark_index);

  // Update all the subsequent landmark index entries on the map (only if
  // it was not a last entry).
  if (static_cast<unsigned int>(erased_landmark_index) <=
      (landmarks_.size() - 1)) {
    for (unsigned int i = erased_landmark_index; i < landmarks_.size(); ++i) {
      LandmarkIdToIdxMap::iterator it;
      it = landmark_id_map_.find(landmarks_[i].id());
      CHECK(it != landmark_id_map_.end());
      it->second = i;
    }
  }
  CHECK_EQ(landmarks_.size(), landmark_id_map_.size());
}

void LandmarkStore::serialize(vi_map::proto::LandmarkStore* proto) const {
  CHECK_NOTNULL(proto);

  const size_t landmarks_size = landmarks_.size();
  google::protobuf::RepeatedPtrField<vi_map::proto::Landmark>* proto_landmarks =
      proto->mutable_landmarks();
  proto_landmarks->Reserve(landmarks_size);
  for (const Landmark& landmark : landmarks_) {
    landmark.serialize(proto_landmarks->Add());
  }
  CHECK_EQ(static_cast<unsigned int>(proto->landmarks_size()), landmarks_size);
}

void LandmarkStore::deserialize(const vi_map::proto::LandmarkStore& proto) {
  landmarks_.resize(proto.landmarks_size());
  for (int i = 0; i < proto.landmarks_size(); ++i) {
    Landmark landmark;
    landmark.deserialize(proto.landmarks(i));

    landmarks_[i] = landmark;
    landmark_id_map_.emplace(landmark.id(), i);
  }
}

} /* namespace vi_map */
