#include "localization-summary-map/localization-summary-map.h"

#include <maplab-common/eigen-proto.h>
#include <maplab-common/file-system-tools.h>
#include <maplab-common/proto-serialization-helper.h>
#include <vi-map/vi-map.h>

#include "localization-summary-map/localization-summary-map.pb.h"

namespace summary_map {

constexpr char LocalizationSummaryMap::kFileName[];

bool LocalizationSummaryMap::operator==(
    const LocalizationSummaryMap& other) const {
  bool is_same = true;
  is_same &= id_ == other.id_;
  is_same &=
      landmark_id_to_landmark_index_ == other.landmark_id_to_landmark_index_;
  is_same &= G_landmark_position_ == other.G_landmark_position_;
  is_same &= G_observer_position_ == other.G_observer_position_;
  is_same &= projected_descriptors_ == other.projected_descriptors_;
  is_same &= observer_indices_ == other.observer_indices_;
  is_same &=
      observation_to_landmark_index_ == other.observation_to_landmark_index_;
  return is_same;
}
bool LocalizationSummaryMap::operator!=(
    const LocalizationSummaryMap& other) const {
  return !(*this == other);
}

void LocalizationSummaryMap::serialize(
    proto::LocalizationSummaryMap* proto) const {
  CHECK_NOTNULL(proto);

  common::eigen_proto::serialize(
      G_landmark_position_, proto->mutable_g_landmark_position());

  // Store the descriptor information as uncompressed localization summary map.
  proto::UncompressedLocalizationSummaryMap* uncompressed_map =
      proto->mutable_uncompressed_map();
  common::eigen_proto::serialize(
      G_observer_position_, uncompressed_map->mutable_g_observer_position());
  common::eigen_proto::serialize(
      projected_descriptors_, uncompressed_map->mutable_descriptors());
  common::eigen_proto::serialize(
      observer_indices_, uncompressed_map->mutable_observer_indices());
  common::eigen_proto::serialize(
      observation_to_landmark_index_,
      uncompressed_map->mutable_observation_to_landmark_index());
}
void LocalizationSummaryMap::deserialize(
    const LocalizationSummaryMapId& localization_summary_map_id,
    const proto::LocalizationSummaryMap& proto) {
  id_ = localization_summary_map_id;

  common::eigen_proto::deserialize(
      proto.g_landmark_position(), &G_landmark_position_);
  initializeLandmarkIds(G_landmark_position_.cols());

  if (proto.has_uncompressed_map()) {
    const proto::UncompressedLocalizationSummaryMap& uncompressed_map =
        proto.uncompressed_map();
    common::eigen_proto::deserialize(
        uncompressed_map.g_observer_position(), &G_observer_position_);

    // Create deterministic IDs based on the loc-summary-map-id.
    constexpr int kMarsennePrime = 131071;
    const int hash_seed = id_.hashToSizeT() ^ kMarsennePrime;
    // Generate arbitrary IDs for the vertices. They are only necessary to
    // communicate with the loop-closure backend and the map.
    for (int i = 0; i < G_observer_position_.cols(); ++i) {
      pose_graph::VertexId vertex_id;
      common::generateIdFromInt(hash_seed + i, &vertex_id);
      CHECK(vertex_id_to_index_.insert(std::make_pair(vertex_id, i)).second)
          << "VertexId collision.";
    }

    common::eigen_proto::deserialize(
        uncompressed_map.descriptors(), &projected_descriptors_);
    common::eigen_proto::deserialize(
        uncompressed_map.observer_indices(), &observer_indices_);
    common::eigen_proto::deserialize(
        uncompressed_map.observation_to_landmark_index(),
        &observation_to_landmark_index_);
  } else {
    LOG(FATAL) << "Unsupported localization summary map format.";
  }
}

bool LocalizationSummaryMap::loadFromFolder(const std::string& folder_path) {
  // Generate a random id.
  LocalizationSummaryMapId summary_map_id;
  common::generateId(&summary_map_id);
  return loadFromFolder(summary_map_id, folder_path);
}

bool LocalizationSummaryMap::loadFromFolder(
    const LocalizationSummaryMapId& summary_map_id,
    const std::string& folder_path) {
  CHECK(!folder_path.empty());
  if (!hasMapOnFileSystem(folder_path)) {
    LOG(ERROR) << "No summary map could be found under \"" << folder_path
               << "\".";
    return false;
  }

  proto::LocalizationSummaryMap proto;
  if (!common::proto_serialization_helper::parseProtoFromFile(
          folder_path, kFileName, &proto)) {
    LOG(ERROR) << "Summary map under \"" << folder_path
               << "\" coulnd't be parsed by protobuf.";
    return false;
  }

  deserialize(summary_map_id, proto);
  return true;
}

bool LocalizationSummaryMap::saveToFolder(
    const std::string& folder_path, const backend::SaveConfig& config) {
  CHECK(!folder_path.empty());
  if (!config.overwrite_existing_files && hasMapOnFileSystem(folder_path)) {
    LOG(ERROR) << "A summary map already exists under \"" << folder_path
               << "\".";
    return false;
  }

  if (!common::createPath(folder_path)) {
    LOG(ERROR) << "Creating path to \"" << folder_path << "\" failed.";
    return false;
  }

  proto::LocalizationSummaryMap proto;
  serialize(&proto);
  return common::proto_serialization_helper::serializeProtoToFile(
      folder_path, kFileName, proto);
}

bool LocalizationSummaryMap::hasMapOnFileSystem(
    const std::string& folder_path) {
  CHECK(!folder_path.empty());
  if (!common::pathExists(folder_path)) {
    return false;
  }
  const std::string complete_file_name = common::concatenateFolderAndFileName(
      common::getRealPath(folder_path), kFileName);
  return common::fileExists(complete_file_name);
}

bool LocalizationSummaryMap::hasLandmark(
    const vi_map::LandmarkId& landmark_id) const {
  return landmark_id_to_landmark_index_.count(landmark_id) > 0u;
}

Eigen::Vector3d LocalizationSummaryMap::getGLandmarkPosition(
    const vi_map::LandmarkId& landmark_id) const {
  std::unordered_map<vi_map::LandmarkId, int>::const_iterator it =
      landmark_id_to_landmark_index_.find(landmark_id);
  CHECK(it != landmark_id_to_landmark_index_.end())
      << "Landmark " << landmark_id << " is not in localization summary map.";
  const int index = it->second;
  CHECK_LT(index, G_landmark_position_.cols());
  return G_landmark_position_.col(index).cast<double>();
}

void LocalizationSummaryMap::getAllObserverIds(
    pose_graph::VertexIdList* observer_ids) const {
  CHECK_NOTNULL(observer_ids);
  observer_ids->resize(vertex_id_to_index_.size());
  for (const std::pair<pose_graph::VertexId, int>& value :
       vertex_id_to_index_) {
    CHECK_LT(value.second, static_cast<int>(observer_ids->size()));
    (*observer_ids)[value.second] = value.first;
  }
}

void LocalizationSummaryMap::getAllLandmarkIds(
    vi_map::LandmarkIdList* landmark_ids) const {
  CHECK_NOTNULL(landmark_ids);
  landmark_ids->resize(landmark_id_to_landmark_index_.size());
  for (const std::pair<vi_map::LandmarkId, int>& value :
       landmark_id_to_landmark_index_) {
    CHECK_LT(value.second, static_cast<int>(landmark_ids->size()));
    (*landmark_ids)[value.second] = value.first;
  }
}

const Eigen::Matrix3Xf& LocalizationSummaryMap::GLandmarkPosition() const {
  return G_landmark_position_;
}
const Eigen::Matrix3Xf& LocalizationSummaryMap::GObserverPosition() const {
  return G_observer_position_;
}
const Eigen::MatrixXf& LocalizationSummaryMap::projectedDescriptors() const {
  return projected_descriptors_;
}
const Eigen::Matrix<unsigned int, Eigen::Dynamic, 1>&
LocalizationSummaryMap::observerIndices() const {
  return observer_indices_;
}
const Eigen::Matrix<unsigned int, Eigen::Dynamic, 1>&
LocalizationSummaryMap::observationToLandmarkIndex() const {
  return observation_to_landmark_index_;
}

void LocalizationSummaryMap::setGLandmarkPosition(
    const Eigen::Matrix3Xd& G_landmark_position) {
  CHECK_GT(G_landmark_position.cols(), 0);

  G_landmark_position_ = G_landmark_position.cast<float>();
  initializeLandmarkIds(G_landmark_position.cols());
}

void LocalizationSummaryMap::initializeLandmarkIds(int num_landmarks) {
  landmark_id_to_landmark_index_.clear();

  // Create deterministic IDs based on the loc-summary-map-id.
  constexpr int kMarsennePrime = 524287;
  const int hash_seed = id_.hashToSizeT() ^ kMarsennePrime;

  for (int i = 0; i < num_landmarks; ++i) {
    vi_map::LandmarkId landmark_id;
    const int id_seed = hash_seed + i;
    common::generateIdFromInt(id_seed, &landmark_id);
    CHECK(
        landmark_id_to_landmark_index_.insert(std::make_pair(landmark_id, i))
            .second)
        << "LandmarkID collision.";
  }
}

void LocalizationSummaryMap::setGObserverPosition(
    const Eigen::Matrix3Xd& G_observer_position) {
  G_observer_position_ = G_observer_position.cast<float>();
}
void LocalizationSummaryMap::setProjectedDescriptors(
    const Eigen::MatrixXf& descriptors) {
  projected_descriptors_ = descriptors;
}
void LocalizationSummaryMap::setObserverIndices(
    const Eigen::Matrix<unsigned int, Eigen::Dynamic, 1>& observer_indices) {
  observer_indices_ = observer_indices;
}
void LocalizationSummaryMap::setObservationToLandmarkIndex(
    const Eigen::Matrix<unsigned int, Eigen::Dynamic, 1>&
        observation_to_landmark_index) {
  observation_to_landmark_index_ = observation_to_landmark_index;
}
}  // namespace summary_map
