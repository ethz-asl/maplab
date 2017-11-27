#include "localization-summary-map/localization-summary-map-creation.h"

#include <fstream>  // NOLINT

#include <Eigen/Core>
#include <descriptor-projection/descriptor-projection.h>
#include <loopclosure-common/flags.h>
#include <loopclosure-common/types.h>
#include <map-sparsification/sampler-base.h>
#include <map-sparsification/sampler-factory.h>
#include <maplab-common/binary-serialization.h>
#include <maplab-common/eigen-proto.h>
#include <vi-map-helpers/vi-map-queries.h>
#include <vi-map/vi-map.h>

#include "localization-summary-map/localization-summary-map-cache.h"
#include "localization-summary-map/localization-summary-map.h"

namespace summary_map {

void createLocalizationSummaryMapForWellConstrainedLandmarks(
    const vi_map::VIMap& map,
    summary_map::LocalizationSummaryMap* summary_map) {
  vi_map_helpers::VIMapQueries queries(map);
  vi_map::LandmarkIdList landmark_ids;
  queries.getAllWellConstrainedLandmarkIds(&landmark_ids);

  createLocalizationSummaryMapFromLandmarkList(
      map, landmark_ids, nullptr, summary_map);
}

void createLocalizationSummaryMapForSummarizedLandmarks(
    const vi_map::VIMap& map, const double landmark_keep_fraction,
    summary_map::LocalizationSummaryMap* summary_map) {
  using map_sparsification::SamplerBase;
  SamplerBase::Ptr sampler = map_sparsification::createSampler(
      SamplerBase::Type::kLpsolvePartitionIlp);

  vi_map_helpers::VIMapQueries queries(map);
  vi_map::LandmarkIdList good_landmark_ids;
  queries.getAllWellConstrainedLandmarkIds(&good_landmark_ids);
  const size_t num_good_landmarks = good_landmark_ids.size();
  const size_t desired_num_landmarks =
      landmark_keep_fraction * num_good_landmarks;

  vi_map::LandmarkIdSet landmarks_to_keep;
  sampler->sample(map, desired_num_landmarks, &landmarks_to_keep);

  vi_map::LandmarkIdList landmarks_to_keep_list(
      landmarks_to_keep.begin(), landmarks_to_keep.end());
  createLocalizationSummaryMapFromLandmarkList(
      map, landmarks_to_keep_list, summary_map);
}

void createLocalizationSummaryMapFromLandmarkList(
    const vi_map::VIMap& map, const vi_map::LandmarkIdList& landmark_ids,
    summary_map::LocalizationSummaryMap* summary_map) {
  createLocalizationSummaryMapFromLandmarkList(
      map, landmark_ids, nullptr, summary_map);
}

void createLocalizationSummaryMapFromLandmarkList(
    const vi_map::VIMap& map,
    const vi_map::LandmarkIdList& landmark_ids,
    LocalizationSummaryMapCache* summary_map_cache,
    summary_map::LocalizationSummaryMap* summary_map) {
  CHECK_NOTNULL(summary_map);
  /// The position of the landmarks in the global frame of reference.
  Eigen::Matrix3Xd G_landmark_position;
  /// The position of the observers in the global frame of reference.
  Eigen::Matrix3Xd G_observer_position;
  /// A set of projected_descriptors from observations of landmarks.
  Eigen::MatrixXf projected_descriptors;
  /// An index of a key-frame for every observation (descriptor).
  Eigen::Matrix<unsigned int, Eigen::Dynamic, 1> observer_indices;
  /// A mapping from observation (descriptor) to index in G_landmark_position.
  Eigen::Matrix<unsigned int, Eigen::Dynamic, 1> observation_to_landmark_index;

  CHECK(!landmark_ids.empty());
  G_landmark_position.resize(Eigen::NoChange, landmark_ids.size());

  std::vector<vi_map::KeypointIdentifier> observations;
  // Best guess reserve.
  observations.reserve(landmark_ids.size() * 4);

  std::vector<unsigned int> observation_to_landmark;
  observation_to_landmark.reserve(landmark_ids.size() * 4);

  // We first collect all observations to the landmarks in question.
  for (size_t landmark_index = 0; landmark_index < landmark_ids.size();
       ++landmark_index) {
    const vi_map::LandmarkId& landmark_id = landmark_ids[landmark_index];
    const vi_map::Landmark& landmark = map.getLandmark(landmark_id);
    G_landmark_position.col(landmark_index) =
        map.getLandmark_G_p_fi(landmark_id);

    const std::vector<vi_map::KeypointIdentifier>& landmark_observations =
        landmark.getObservations();
    observations.insert(
        observations.end(), landmark_observations.begin(),
        landmark_observations.end());

    // Push the index of the landmark for all the observations.
    for (size_t i = 0; i < landmark_observations.size(); ++i) {
      observation_to_landmark.push_back(landmark_index);
    }
  }
  CHECK(!observations.empty()) << "No landmark observations for summary map.";

  // Copy all the observation to landmark indices into the summary-map format.
  observation_to_landmark_index =
      Eigen::Map<const Eigen::Matrix<unsigned int, Eigen::Dynamic, 1> >(
          observation_to_landmark.data(), observation_to_landmark.size(), 1);

  const char* loop_closure_files_path = getenv("MAPLAB_LOOPCLOSURE_DIR");
  CHECK_NE(loop_closure_files_path, static_cast<char*>(NULL))
      << "MAPLAB_LOOPCLOSURE_DIR environment variable is not set.\n"
         "Source the MapLab environment from your workspace:\n"
         "  . devel/setup.bash";

  if (FLAGS_feature_descriptor_type == loop_closure::kFeatureDescriptorFREAK) {
    if (FLAGS_lc_projection_matrix_filename == "") {
      FLAGS_lc_projection_matrix_filename =
          std::string(loop_closure_files_path) + "/projection_matrix_freak.dat";
    }
  } else {
    if (FLAGS_lc_projection_matrix_filename == "") {
      FLAGS_lc_projection_matrix_filename =
          std::string(loop_closure_files_path) + "/projection_matrix_brisk.dat";
    }
  }
  Eigen::MatrixXf projection_matrix;
  std::ifstream deserializer(FLAGS_lc_projection_matrix_filename);
  CHECK(deserializer.is_open()) << "Cannot load projection matrix from file: "
                                << FLAGS_lc_projection_matrix_filename;
  common::Deserialize(&projection_matrix, &deserializer);

  projected_descriptors.resize(
      FLAGS_lc_target_dimensionality, observations.size());
  observer_indices.resize(observations.size());
  Aligned<std::vector, Eigen::Vector3d> G_observer_positions;

  std::unordered_map<vi_map::VisualFrameIdentifier, int> frame_id_to_index;
  int observer_index = 0;
  for (size_t observation_index = 0; observation_index < observations.size();
       ++observation_index) {
    // We store the observer index for covisibility graph based filtering.
    const vi_map::KeypointIdentifier& observation =
        observations[observation_index];
    std::unordered_map<vi_map::VisualFrameIdentifier, int>::const_iterator it =
        frame_id_to_index.find(observation.frame_id);
    if (it == frame_id_to_index.end()) {
      it = frame_id_to_index
               .insert(std::make_pair(observation.frame_id, observer_index))
               .first;
      Eigen::Vector3d G_observer_position =
          map.getVertex_G_p_I(observation.frame_id.vertex_id);
      G_observer_positions.push_back(G_observer_position);
      ++observer_index;
    }
    observer_indices(observation_index, 0) = it->second;

    CHECK_LT(observation_index, observation_to_landmark.size());
    const size_t landmark_index = observation_to_landmark[observation_index];
    CHECK_LT(landmark_index, landmark_ids.size());
    const vi_map::LandmarkId& landmark_id = landmark_ids[landmark_index];
    if (summary_map_cache == nullptr ||
        !summary_map_cache->getProjectedDescriptorForLandmark(
            observation, landmark_id,
            projected_descriptors.col(observation_index))) {
      // No projected descriptor is stored yet, need to compute first.
      const aslam::VisualFrame& frame =
          map.getVertex(observation.frame_id.vertex_id)
              .getVisualFrame(observation.frame_id.frame_index);

      Eigen::Map<const Eigen::Matrix<unsigned char, Eigen::Dynamic, 1> >
          raw_descriptor(
              frame.getDescriptor(observation.keypoint_index),
              frame.getDescriptorSizeBytes(), 1);

      // Project the descriptors directly into the descriptor storage.
      descriptor_projection::ProjectDescriptor(
          raw_descriptor, projection_matrix, FLAGS_lc_target_dimensionality,
          projected_descriptors.col(observation_index));

      if (summary_map_cache != nullptr) {
        summary_map_cache->addProjectedDescriptor(
            observation, landmark_id,
            projected_descriptors.col(observation_index));
      }
    }
  }
  G_observer_position.resize(Eigen::NoChange, G_observer_positions.size());
  for (size_t i = 0; i < G_observer_positions.size(); ++i) {
    G_observer_position.col(i) = G_observer_positions[i];
  }

  summary_map->setGLandmarkPosition(G_landmark_position);
  summary_map->setGObserverPosition(G_observer_position);
  summary_map->setProjectedDescriptors(projected_descriptors);
  summary_map->setObserverIndices(observer_indices);
  summary_map->setObservationToLandmarkIndex(observation_to_landmark_index);
}
}  // namespace summary_map
