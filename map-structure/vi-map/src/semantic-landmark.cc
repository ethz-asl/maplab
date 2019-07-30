#include "vi-map/semantic-landmark.h"

#include <algorithm>
#include <utility>

#include <maplab-common/eigen-proto.h>

namespace vi_map {
double* SemanticLandmark::get_p_B_Mutable() {
  return B_position_.data();
}

double* SemanticLandmark::get_p_B_CovarianceMutable() {
  // Allocate memory for the covariance if it wasn't yet set.
  if (B_covariance_ == nullptr) {
    B_covariance_ = aligned_unique<Eigen::Matrix3d>();
    B_covariance_->setIdentity();
  }
  return B_covariance_->data();
}

void SemanticLandmark::addObservation(
    const pose_graph::VertexId& vertex_id, unsigned int frame_idx,
    unsigned int measurement_index) {
  SemanticObjectIdentifier backlink;
  backlink.frame_id.vertex_id = vertex_id;
  backlink.frame_id.frame_index = frame_idx;
  backlink.measurement_index = measurement_index;
  observations_.push_back(backlink);
}

void SemanticLandmark::addObservation(const SemanticObjectIdentifier& new_observation) {
  observations_.push_back(new_observation);
}

void SemanticLandmark::addObservations(const SemanticObjectIdentifierList& new_observations) {
  observations_.insert(
      observations_.end(), new_observations.begin(), new_observations.end());
}

bool SemanticLandmark::hasObservation(
    const pose_graph::VertexId& vertex_id, size_t frame_idx,
    size_t measurement_index) const {
  for (const SemanticObjectIdentifier& backlink : observations_) {
    if (backlink.frame_id.vertex_id == vertex_id &&
        backlink.frame_id.frame_index == frame_idx &&
        backlink.measurement_index == measurement_index) {
      return true;
    }
  }
  return false;
}

void SemanticLandmark::removeAllObservationsAccordingToPredicate(
    const std::function<bool(const SemanticObjectIdentifier&)>& // NOLINT
        predicate) {
  SemanticObjectIdentifierList::iterator observation_iterator = observations_.begin();
  while (observation_iterator != observations_.end()) {
    if (predicate(*observation_iterator)) {
      observation_iterator = observations_.erase(observation_iterator);
    } else {
      ++observation_iterator;
    }
  }
}

void SemanticLandmark::removeObservation(const SemanticObjectIdentifier& observation) {
  std::function<bool(const SemanticObjectIdentifier& observation)> // NOLINT
      predicate = [&](const SemanticObjectIdentifier& inspected_observation) {
        return inspected_observation == observation;
      };
  removeAllObservationsAccordingToPredicate(predicate);
}

void SemanticLandmark::removeAllObservationsOfVertex(
    const pose_graph::VertexId& vertex_id) {
  std::function<bool(const SemanticObjectIdentifier& observation)> // NOLINT
      predicate =
      [&](const SemanticObjectIdentifier& observation) {
        return observation.frame_id.vertex_id == vertex_id;
      };
  removeAllObservationsAccordingToPredicate(predicate);
}

void SemanticLandmark::removeAllObservationsOfVertexAndFrame(
    const pose_graph::VertexId& vertex_id, unsigned int frame_idx) {
  std::function<bool(const SemanticObjectIdentifier& observation)> // NOLINT
      predicate =
      [&](const SemanticObjectIdentifier& observation) {
        return (observation.frame_id.vertex_id == vertex_id) &&
               (observation.frame_id.frame_index == frame_idx);
      };
  removeAllObservationsAccordingToPredicate(predicate);
}

unsigned int SemanticLandmark::numberOfObserverVertices() const {
  pose_graph::VertexIdSet vertices;
  for (const SemanticObjectIdentifier& backlink : observations_) {
    vertices.insert(backlink.frame_id.vertex_id);
  }
  return vertices.size();
}

bool SemanticLandmark::hasObservation(const SemanticObjectIdentifier& id) const {
  return std::find(observations_.begin(), observations_.end(), id) !=
         observations_.end();
}

const SemanticObjectIdentifierList& SemanticLandmark::getObservations() const {
  return observations_;
}

unsigned int SemanticLandmark::numberOfObservations() const {
  return observations_.size();
}

bool SemanticLandmark::hasObservations() const {
  return !observations_.empty();
}

void SemanticLandmark::forEachObservation(
    const std::function<void(const SemanticObjectIdentifier&)>& action) const {
  for (const SemanticObjectIdentifier& observation : observations_) {
    action(observation);
  }
}

void SemanticLandmark::forEachObservation(
    const std::function<void(const SemanticObjectIdentifier&, const size_t)>& action)
    const {
  for (size_t i = 0u; i < observations_.size(); ++i) {
    action(observations_[i], i);
  }
}


void SemanticLandmark::serialize(vi_map::proto::SemanticLandmark* proto) const {
  CHECK_NOTNULL(proto);

  proto->Clear();
  id_.serialize(proto->mutable_id());
  proto->set_class_id(class_id_);

  {
    const size_t num_observations = observations_.size();
    google::protobuf::RepeatedPtrField<common::proto::Id>* vertex_ids_proto =
        proto->mutable_vertex_ids();
    google::protobuf::RepeatedField<google::protobuf::uint32>*
        frame_indices_proto = proto->mutable_frame_indices();
    google::protobuf::RepeatedField<google::protobuf::uint32>*
        measurement_indices_proto = proto->mutable_measurement_indices();
    vertex_ids_proto->Reserve(num_observations);
    frame_indices_proto->Reserve(num_observations);
    measurement_indices_proto->Reserve(num_observations);
    for (unsigned int i = 0u; i < num_observations; ++i) {
      const SemanticObjectIdentifier& observation = observations_[i];
      observation.frame_id.vertex_id.serialize(vertex_ids_proto->Add());
      frame_indices_proto->Add(observation.frame_id.frame_index);
      measurement_indices_proto->Add(observation.measurement_index);
    }
  }

  common::eigen_proto::serialize(B_position_, proto->mutable_position());
  // Serialize the covariance if it is set.
  if (B_covariance_ != nullptr) {
    common::eigen_proto::serialize(*B_covariance_, proto->mutable_covariance());
  }

  switch (quality_) {
    case Quality::kUnknown: {
      proto->set_quality(::vi_map::proto::SemanticLandmark_Quality_kUnknown);
      break;
    }
    case Quality::kBad: {
      proto->set_quality(::vi_map::proto::SemanticLandmark_Quality_kBad);
      break;
    }
    case Quality::kGood: {
      proto->set_quality(::vi_map::proto::SemanticLandmark_Quality_kGood);
      break;
    }
    default: {
      LOG(FATAL) << "Unknown Semanticlandmark quality value "
                 << static_cast<int>(quality_);
    }
  }
}

void SemanticLandmark::clearObservations() {
  observations_.clear();
}

void SemanticLandmark::deserialize(const vi_map::proto::SemanticLandmark& proto) {
  id_.deserialize(proto.id());
  class_id_ = proto.class_id();
  CHECK_EQ(proto.vertex_ids_size(), proto.measurement_indices_size());

  observations_.resize(proto.measurement_indices_size());
  for (int i = 0; i < proto.measurement_indices_size(); ++i) {
    SemanticObjectIdentifier backlink;
    backlink.frame_id.vertex_id.deserialize(proto.vertex_ids(i));
    backlink.measurement_index = proto.measurement_indices(i);

    CHECK_GT(proto.frame_indices_size(), 0)
        << "Couldn't deserialize the frame indices for the measurements. Possibly "
        << "your map format is outdated.";
    backlink.frame_id.frame_index = proto.frame_indices(i);
    observations_[i] = backlink;
  }

  if (proto.has_quality()) {
    quality_ = static_cast<Quality>(proto.quality());
    switch (proto.quality()) {
      case ::vi_map::proto::SemanticLandmark_Quality_kUnknown: {
        quality_ = Quality::kUnknown;
        break;
      }
      case ::vi_map::proto::SemanticLandmark_Quality_kBad: {
        quality_ = Quality::kBad;
        break;
      }
      case ::vi_map::proto::SemanticLandmark_Quality_kGood: {
        quality_ = Quality::kGood;
        break;
      }
      default: {
        LOG(FATAL) << "Unknown Semanticlandmark quality value "
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
