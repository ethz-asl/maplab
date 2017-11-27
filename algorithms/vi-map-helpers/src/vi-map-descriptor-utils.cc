#include "vi-map-helpers/vi-map-descriptor-utils.h"

#include <vector>

#include <aslam/common/descriptor-utils.h>
#include <aslam/common/feature-descriptor-ref.h>
#include <aslam/frames/visual-nframe.h>
#include <glog/logging.h>
#include <maplab-common/progress-bar.h>

namespace vi_map_helpers {

void getDescriptorFromObservation(
    const vi_map::KeypointIdentifier& observation, const vi_map::VIMap& map,
    Eigen::Matrix<unsigned char, Eigen::Dynamic, 1>* descriptor) {
  CHECK_NOTNULL(descriptor);
  const pose_graph::VertexId& vertex_id = observation.frame_id.vertex_id;
  CHECK(map.hasVertex(vertex_id));
  const vi_map::Vertex& vertex = map.getVertex(vertex_id);
  const size_t frame_index = observation.frame_id.frame_index;
  CHECK_LT(frame_index, vertex.numFrames());
  const aslam::VisualFrame& visual_frame = vertex.getVisualFrame(frame_index);
  const size_t keypoint_index = observation.keypoint_index;
  const aslam::VisualFrame::DescriptorsT& descriptors =
      visual_frame.getDescriptors();
  CHECK_LT(keypoint_index, static_cast<size_t>(descriptors.cols()));
  *descriptor = descriptors.col(keypoint_index);
}

void getDescriptorClosestToMedian(
    const vi_map::KeypointIdentifierList& observations,
    const vi_map::VIMap& map,
    Eigen::Matrix<unsigned char, Eigen::Dynamic, 1>* descriptor,
    size_t* descriptor_index) {
  CHECK_NOTNULL(descriptor);
  CHECK_NOTNULL(descriptor_index);

  const size_t num_observations = observations.size();
  if (num_observations == 0u) {
    return;
  }

  // Retrieve descriptor size.
  const pose_graph::VertexId& vertex_id = observations[0].frame_id.vertex_id;
  CHECK(map.hasVertex(vertex_id));
  const vi_map::Vertex& vertex = map.getVertex(vertex_id);
  CHECK_GT(vertex.numFrames(), 0u);
  const aslam::VisualFrame& visual_frame = vertex.getVisualFrame(0u);
  const size_t descriptor_size_bytes = visual_frame.getDescriptorSizeBytes();
  CHECK_GT(descriptor_size_bytes, 0u);

  if (num_observations == 1u) {
    getDescriptorFromObservation(observations[0], map, descriptor);
    return;
  }

  aslam::VisualFrame::DescriptorsT raw_descriptors =
      aslam::VisualFrame::DescriptorsT::Zero(
          descriptor_size_bytes, num_observations);

  for (size_t observation_idx = 0u; observation_idx < num_observations;
       ++observation_idx) {
    const vi_map::KeypointIdentifier& observation =
        observations[observation_idx];
    const pose_graph::VertexId& vertex_id = observation.frame_id.vertex_id;
    CHECK(map.hasVertex(vertex_id));
    const vi_map::Vertex& vertex = map.getVertex(vertex_id);
    CHECK_GT(vertex.numFrames(), 0u);
    const aslam::VisualFrame& visual_frame =
        vertex.getVisualFrame(observation.frame_id.frame_index);

    CHECK_LT(
        observation.keypoint_index,
        static_cast<size_t>(visual_frame.getDescriptors().cols()));
    raw_descriptors.col(observation_idx) =
        visual_frame.getDescriptors().col(observation.keypoint_index);
  }

  size_t median_descriptor_index = 0u;
  aslam::common::descriptor_utils::getIndexOfDescriptorClosestToMedian(
      raw_descriptors, &median_descriptor_index);
  CHECK_LT(median_descriptor_index, num_observations);

  getDescriptorFromObservation(
      observations[median_descriptor_index], map, descriptor);
}

}  // namespace vi_map_helpers
