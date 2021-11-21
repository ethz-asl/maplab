#include "vi-map/landmark.h"

#include <algorithm>
#include <utility>

#include <maplab-common/eigen-proto.h>

namespace vi_map {

double* Landmark::get_p_B_Mutable() {
  return B_position_.data();
}

double* Landmark::get_p_B_CovarianceMutable() {
  // Allocate memory for the covariance if it wasn't yet set.
  if (B_covariance_ == nullptr) {
    B_covariance_ = aligned_unique<Eigen::Matrix3d>();
    B_covariance_->setIdentity();
  }
  return B_covariance_->data();
}

void Landmark::addObservation(
    const pose_graph::VertexId& vertex_id, unsigned int frame_idx,
    unsigned int keypoint_index) {
  KeypointIdentifier backlink;
  backlink.frame_id.vertex_id = vertex_id;
  backlink.frame_id.frame_index = frame_idx;
  backlink.keypoint_index = keypoint_index;
  observations_.push_back(backlink);
}

void Landmark::addObservation(const KeypointIdentifier& keypoint_id) {
  observations_.push_back(keypoint_id);
}

void Landmark::addObservations(const KeypointIdentifierList& new_observations) {
  observations_.insert(
      observations_.end(), new_observations.begin(), new_observations.end());
}

bool Landmark::hasObservation(
    const pose_graph::VertexId& vertex_id, size_t frame_idx,
    size_t keypoint_index) const {
  for (const KeypointIdentifier& backlink : observations_) {
    if (backlink.frame_id.vertex_id == vertex_id &&
        backlink.frame_id.frame_index == frame_idx &&
        backlink.keypoint_index == keypoint_index) {
      return true;
    }
  }
  return false;
}

void Landmark::removeAllObservationsAccordingToPredicate(
    const std::function<bool(const KeypointIdentifier&)>&  // NOLINT
        predicate) {
  KeypointIdentifierList::iterator observation_iterator = observations_.begin();
  while (observation_iterator != observations_.end()) {
    if (predicate(*observation_iterator)) {
      observation_iterator = observations_.erase(observation_iterator);
    } else {
      ++observation_iterator;
    }
  }
}

void Landmark::removeObservation(const KeypointIdentifier& observation) {
  std::function<bool(const KeypointIdentifier& observation)>  // NOLINT
      predicate = [&](const KeypointIdentifier& inspected_observation) {
        return inspected_observation == observation;
      };
  removeAllObservationsAccordingToPredicate(predicate);
}

void Landmark::removeAllObservationsOfVertex(
    const pose_graph::VertexId& vertex_id) {
  std::function<bool(const KeypointIdentifier& observation)>  // NOLINT
      predicate = [&](const KeypointIdentifier& observation) {
        return observation.frame_id.vertex_id == vertex_id;
      };
  removeAllObservationsAccordingToPredicate(predicate);
}

void Landmark::removeAllObservationsOfVertexAndFrame(
    const pose_graph::VertexId& vertex_id, unsigned int frame_idx) {
  std::function<bool(const KeypointIdentifier& observation)>  // NOLINT
      predicate = [&](const KeypointIdentifier& observation) {
        return (observation.frame_id.vertex_id == vertex_id) &&
               (observation.frame_id.frame_index == frame_idx);
      };
  removeAllObservationsAccordingToPredicate(predicate);
}

unsigned int Landmark::numberOfObserverVertices() const {
  pose_graph::VertexIdSet vertices;
  for (const KeypointIdentifier& backlink : observations_) {
    vertices.insert(backlink.frame_id.vertex_id);
  }
  return vertices.size();
}

bool Landmark::hasObservation(const KeypointIdentifier& keypoint_id) const {
  return std::find(observations_.begin(), observations_.end(), keypoint_id) !=
         observations_.end();
}

const KeypointIdentifierList& Landmark::getObservations() const {
  return observations_;
}

unsigned int Landmark::numberOfObservations() const {
  return observations_.size();
}

bool Landmark::hasObservations() const {
  return !observations_.empty();
}

void Landmark::forEachObservation(
    const std::function<void(const KeypointIdentifier&)>& action) const {
  for (const KeypointIdentifier& observation : observations_) {
    action(observation);
  }
}

void Landmark::forEachObservation(
    const std::function<void(const KeypointIdentifier&, const size_t)>& action)
    const {
  for (size_t i = 0u; i < observations_.size(); ++i) {
    action(observations_[i], i);
  }
}

void Landmark::serialize(vi_map::proto::Landmark* proto) const {
  CHECK_NOTNULL(proto);

  proto->Clear();
  id_.serialize(proto->mutable_id());

  {
    const size_t num_observations = observations_.size();
    google::protobuf::RepeatedPtrField<aslam::proto::Id>* vertex_ids_proto =
        proto->mutable_vertex_ids();
    google::protobuf::RepeatedField<google::protobuf::uint32>*
        frame_indices_proto = proto->mutable_frame_indices();
    google::protobuf::RepeatedField<google::protobuf::uint32>*
        keypoint_indices_proto = proto->mutable_keypoint_indices();
    vertex_ids_proto->Reserve(num_observations);
    frame_indices_proto->Reserve(num_observations);
    keypoint_indices_proto->Reserve(num_observations);
    for (unsigned int i = 0u; i < num_observations; ++i) {
      const KeypointIdentifier& observation = observations_[i];
      observation.frame_id.vertex_id.serialize(vertex_ids_proto->Add());
      frame_indices_proto->Add(observation.frame_id.frame_index);
      keypoint_indices_proto->Add(observation.keypoint_index);
    }
  }

  common::eigen_proto::serialize(B_position_, proto->mutable_position());
  // Serialize the covariance if it is set.
  if (B_covariance_ != nullptr) {
    common::eigen_proto::serialize(*B_covariance_, proto->mutable_covariance());
  }

  switch (quality_) {
    case Quality::kUnknown: {
      proto->set_quality(::vi_map::proto::Landmark_Quality_kUnknown);
      break;
    }
    case Quality::kBad: {
      proto->set_quality(::vi_map::proto::Landmark_Quality_kBad);
      break;
    }
    case Quality::kGood: {
      proto->set_quality(::vi_map::proto::Landmark_Quality_kGood);
      break;
    }
    default: {
      LOG(FATAL) << "Unknown landmark quality value "
                 << static_cast<int>(quality_);
    }
  }
}

void Landmark::clearObservations() {
  observations_.clear();
}

void Landmark::deserialize(const vi_map::proto::Landmark& proto) {
  id_.deserialize(proto.id());

  CHECK_EQ(proto.vertex_ids_size(), proto.keypoint_indices_size());

  observations_.resize(proto.keypoint_indices_size());
  for (int i = 0; i < proto.keypoint_indices_size(); ++i) {
    KeypointIdentifier backlink;
    backlink.frame_id.vertex_id.deserialize(proto.vertex_ids(i));
    backlink.keypoint_index = proto.keypoint_indices(i);

    CHECK_GT(proto.frame_indices_size(), 0)
        << "Couldn't deserialize the frame indices for the keypoints. Possibly "
        << "your map format is outdated.";
    backlink.frame_id.frame_index = proto.frame_indices(i);
    observations_[i] = backlink;
  }

  if (proto.has_quality()) {
    quality_ = static_cast<Quality>(proto.quality());
    switch (proto.quality()) {
      case ::vi_map::proto::Landmark_Quality_kUnknown: {
        quality_ = Quality::kUnknown;
        break;
      }
      case ::vi_map::proto::Landmark_Quality_kBad: {
        quality_ = Quality::kBad;
        break;
      }
      case ::vi_map::proto::Landmark_Quality_kGood: {
        quality_ = Quality::kGood;
        break;
      }
      default: {
        LOG(FATAL) << "Unknown landmark quality value "
                   << static_cast<int>(proto.quality());
      }
    }
  } else {
    quality_ = Quality::kUnknown;
  }

  common::eigen_proto::deserialize(proto.position(), &B_position_);

  // Deserialize the covariance if it is set.
  if (proto.covariance_size() > 0) {
    // Allocate the covariance memory if needed.
    if (B_covariance_ == nullptr) {
      B_covariance_ = aligned_unique<Eigen::Matrix3d>();
    }
    common::eigen_proto::deserialize(
        proto.covariance(), CHECK_NOTNULL(B_covariance_.get()));
  }
}
}  // namespace vi_map
