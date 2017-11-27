#include <string>
#include <vector>

#include <Eigen/Core>
#include <aslam/common/feature-descriptor-ref.h>
#include <descriptor-projection/descriptor-projection.h>
#include <descriptor-projection/map-track-extractor.h>
#include <loopclosure-common/types.h>
#include <vi-map/vi-map.h>
#include <vocabulary-tree/distance.h>

namespace descriptor_projection {
void CollectAndConvertDescriptors(
    const vi_map::VIMap& map, const vi_map::MissionId& mission_id,
    unsigned int descriptor_size_bits, unsigned int matching_threshold,
    Eigen::MatrixXf* all_descriptors, std::vector<Track>* tracks) {
  CHECK_NOTNULL(all_descriptors);
  CHECK_NOTNULL(tracks);
  loop_closure::DescriptorContainer all_descriptors_char;
  CollectAndConvertDescriptors(
      map, mission_id, descriptor_size_bits, matching_threshold,
      &all_descriptors_char, tracks);

  CHECK_EQ(descriptor_size_bits / 8, all_descriptors_char.rows());

  all_descriptors->resize(descriptor_size_bits, all_descriptors_char.cols());

  for (int i = 0; i < all_descriptors_char.cols(); ++i) {
    DescriptorToEigenMatrix(
        all_descriptors_char.col(i), all_descriptors->col(i));
  }
}
void CollectAndConvertDescriptors(
    const vi_map::VIMap& map, const vi_map::MissionId& mission_id,
    unsigned int descriptor_size_bits, unsigned int matching_threshold,
    loop_closure::DescriptorContainer* all_descriptors,
    std::vector<Track>* tracks) {
  CHECK_NOTNULL(all_descriptors);
  CHECK_NOTNULL(tracks);

  vi_map::LandmarkIdList all_landmark_ids;
  map.getAllLandmarkIdsInMission(mission_id, &all_landmark_ids);

  // First count the total number of descriptors contained in all frames.
  unsigned int total_number_of_descriptors = 0;
  for (const vi_map::LandmarkId& landmark_id : all_landmark_ids) {
    const vi_map::Landmark& landmark = map.getLandmark(landmark_id);
    const int number_of_observations = landmark.getObservations().size();
    total_number_of_descriptors += number_of_observations;
  }

  LOG(INFO) << "Got " << total_number_of_descriptors << " descriptors.";

  const int num_descriptor_bytes = descriptor_size_bits / 8;
  all_descriptors->resize(num_descriptor_bytes, total_number_of_descriptors);

  // Go through all landmarks and get their observations, to determine pairs of
  // matching descriptors.
  using aslam::common::FeatureDescriptorConstRef;
  loop_closure::distance::Hamming<FeatureDescriptorConstRef> hamming;

  unsigned int current_descriptor_idx = 0;
  for (const vi_map::LandmarkId& landmark_id : all_landmark_ids) {
    const vi_map::Landmark& landmark = map.getLandmark(landmark_id);
    const vi_map::KeypointIdentifierList& observations =
        landmark.getObservations();
    int number_of_observations = observations.size();

    // Assemble the tracks of landmark observations.
    tracks->emplace_back(Track());
    Track& track = tracks->back();
    track.reserve(number_of_observations);

    if (observations.empty()) {
      continue;
    }

    // Get the first descriptor of this track.
    FeatureDescriptorConstRef last_descriptor(nullptr, 0);

    const vi_map::KeypointIdentifier& first_observation = observations[0];
    const vi_map::Vertex& first_observer_vertex =
        map.getVertex(first_observation.frame_id.vertex_id);
    const aslam::VisualFrame& first_observer_frame =
        first_observer_vertex.getVisualFrame(
            first_observation.frame_id.frame_index);

    last_descriptor = aslam::common::FeatureDescriptorConstRef(
        first_observer_frame.getDescriptor(first_observation.keypoint_index),
        first_observer_frame.getDescriptorSizeBytes());

    CHECK_NOTNULL(last_descriptor.data());

    // Go through all observations.
    for (const vi_map::KeypointIdentifier& observation : observations) {
      const vi_map::Vertex& observer_vertex =
          map.getVertex(observation.frame_id.vertex_id);
      const aslam::VisualFrame& observer_frame =
          observer_vertex.getVisualFrame(observation.frame_id.frame_index);

      const int descriptor_bytes = observer_frame.getDescriptorSizeBytes();

      all_descriptors->block(0, current_descriptor_idx, descriptor_bytes, 1) =
          Eigen::Map<const Eigen::Matrix<unsigned char, Eigen::Dynamic, 1> >(
              observer_frame.getDescriptor(observation.keypoint_index),
              descriptor_bytes, 1);

      aslam::common::FeatureDescriptorConstRef current_descriptor(
          observer_frame.getDescriptor(observation.keypoint_index),
          descriptor_bytes);

      // Only add the descriptor as track if it is a close match.
      unsigned int distance = hamming(current_descriptor, last_descriptor);

      if (distance < matching_threshold) {
        track.push_back(current_descriptor_idx);
      }
      last_descriptor = current_descriptor;
      ++current_descriptor_idx;
    }
  }
}
}  // namespace descriptor_projection
