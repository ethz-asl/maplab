#include <vi-map/semantic-landmark-store.h>

#include <glog/logging.h>

namespace vi_map {

double* SemanticLandmarkStore::get_p_B_Mutable(const SemanticLandmarkId& landmark_id) {
  SemanticLandmarkIdToIdxMap::const_iterator it;
  it = landmark_id_map_.find(landmark_id);
  CHECK(it != landmark_id_map_.end());

  return landmarks_[it->second].get_p_B_Mutable();
}

SemanticLandmark& SemanticLandmarkStore::getSemanticLandmark(const SemanticLandmarkId& landmark_id) {
  SemanticLandmarkIdToIdxMap::const_iterator it;
  it = landmark_id_map_.find(landmark_id);
  CHECK(it != landmark_id_map_.end());

  return landmarks_[it->second];
}

const SemanticLandmark& SemanticLandmarkStore::getSemanticLandmark(
    const SemanticLandmarkId& landmark_id) const {
  SemanticLandmarkIdToIdxMap::const_iterator it;
  it = landmark_id_map_.find(landmark_id);
  CHECK(it != landmark_id_map_.end());
  CHECK_LT(static_cast<unsigned int>(it->second), landmarks_.size());
  return landmarks_[it->second];
}

void SemanticLandmarkStore::addSemanticLandmark(const SemanticLandmark& landmark) {
  const unsigned int idx = landmarks_.size();
  landmarks_.push_back(landmark);
  landmark_id_map_.insert(std::make_pair(landmark.id(), idx));
  CHECK_EQ(landmarks_.size(), landmark_id_map_.size());
}

bool SemanticLandmarkStore::hasSemanticLandmark(const SemanticLandmarkId& landmark_id) const {
  return landmark_id_map_.count(landmark_id);
}

unsigned int SemanticLandmarkStore::size() const {
  return landmarks_.size();
}

void SemanticLandmarkStore::removeSemanticLandmark(const SemanticLandmarkId& landmark_id) {
  SemanticLandmarkIdToIdxMap::const_iterator it;
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
      SemanticLandmarkIdToIdxMap::iterator it;
      it = landmark_id_map_.find(landmarks_[i].id());
      CHECK(it != landmark_id_map_.end());
      it->second = i;
    }
  }
  CHECK_EQ(landmarks_.size(), landmark_id_map_.size());
}

void SemanticLandmarkStore::serialize(vi_map::proto::SemanticLandmarkStore* proto) const {
  CHECK_NOTNULL(proto);

  const size_t landmarks_size = landmarks_.size();
  google::protobuf::RepeatedPtrField<vi_map::proto::SemanticLandmark>* proto_semantic_landmarks =
      proto->mutable_semantic_landmarks();
  proto_semantic_landmarks->Reserve(landmarks_size);
  for (const SemanticLandmark& landmark : landmarks_) {
    landmark.serialize(proto_semantic_landmarks->Add());
  }
  CHECK_EQ(static_cast<unsigned int>(proto->semantic_landmarks_size()), landmarks_size);
}

void SemanticLandmarkStore::deserialize(const vi_map::proto::SemanticLandmarkStore& proto) {
  landmarks_.resize(proto.semantic_landmarks_size());
  for (int i = 0; i < proto.semantic_landmarks_size(); ++i) {
    SemanticLandmark landmark;
    landmark.deserialize(proto.semantic_landmarks(i));

    landmarks_[i] = landmark;
    landmark_id_map_.emplace(landmark.id(), i);
  }
}

} /* namespace vi_map */
