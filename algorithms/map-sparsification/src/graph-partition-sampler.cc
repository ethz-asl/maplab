#include "map-sparsification/graph-partition-sampler.h"

#include <aslam/common/timer.h>
#include <glog/logging.h>
#include <vi-map-helpers/vi-map-partitioner.h>
#include <vi-map-helpers/vi-map-queries.h>
#include <visualization/color-palette.h>

#include "map-sparsification/visualization/map-sparsification-visualization.h"

namespace map_sparsification {

GraphPartitionSampler::GraphPartitionSampler(
    map_sparsification::SamplerBase::Ptr sampler)
    : sampler_(sampler), max_partitioned_summarization_fraction_(1.0) {
  CHECK(sampler_);
}

GraphPartitionSampler::~GraphPartitionSampler() {}

void GraphPartitionSampler::partitionMapIfNecessary(const vi_map::VIMap& map) {
  vi_map_helpers::VIMapQueries queries(map);
  vi_map::LandmarkIdList well_constrained_landmarks;
  queries.getAllWellConstrainedLandmarkIds(&well_constrained_landmarks);

  const size_t num_landmarks = well_constrained_landmarks.size();
  if (num_landmarks < kMaxNumLandmarkPerPartition) {
    posegraph_partitioning_.resize(1u);
    map.getAllVertexIds(&(posegraph_partitioning_[0]));
  } else {
    const size_t num_partitions = std::ceil(
        static_cast<double>(num_landmarks) / kMaxNumLandmarkPerPartition);
    LOG(INFO) << "Number of well constrained landmarks exceeds "
              << kMaxNumLandmarkPerPartition << ". Will partition the graph "
              << "into " << num_partitions << " partitions.";
    vi_map_helpers::VIMapPartitioner partitioner;
    partitioner.partitionMapWithMetis(
        map, num_partitions, &posegraph_partitioning_);
    CHECK_EQ(num_partitions, posegraph_partitioning_.size());
  }
}

void GraphPartitionSampler::setMaxPartitionedSummarizationFraction(
    double fraction) {
  CHECK_GE(fraction, 0.0);
  CHECK_LE(fraction, 1.0);
  max_partitioned_summarization_fraction_ = fraction;
}

void GraphPartitionSampler::sample(
    const vi_map::VIMap& map, unsigned int total_desired_num_landmarks,
    vi_map::LandmarkIdSet* summary_landmark_ids) {
  CHECK_NOTNULL(summary_landmark_ids)->clear();

  const size_t num_store_landmarks = map.numLandmarks();
  if (num_store_landmarks == 0) {
    LOG(WARNING) << "No landmarks in the map, bailing out early.";
  }

  double retain_ratio =
      static_cast<double>(total_desired_num_landmarks) / num_store_landmarks;

  if ((1.0 - retain_ratio) > max_partitioned_summarization_fraction_) {
    LOG(WARNING) << "Too high summarization fraction, will perform a global "
                 << "optimization after summarizing segments.";
    retain_ratio = 1.0 - max_partitioned_summarization_fraction_;
  }

  partitionMapIfNecessary(map);

  // Reset plotting data.
  if (visualizer_) {
    partition_landmarks_.clear();
    const bool kGloballySelectedLandmarks = false;
    vi_map::LandmarkIdSet empty_set;
    visualizer_->plotLandmarks(map, -1, empty_set, posegraph_partitioning_,
                               partition_landmarks_,
                               kGloballySelectedLandmarks);
  }
  partition_landmarks_.resize(posegraph_partitioning_.size());

  // TODO(dymczykm) Parallelize this.
  for (unsigned int i = 0; i < posegraph_partitioning_.size(); ++i) {
    LOG(INFO) << "Sampling cluster " << (i + 1) << " of "
              << posegraph_partitioning_.size();
    if (visualizer_) {
      visualizer_->plotSegment(map, posegraph_partitioning_, i);
    }

    vi_map::LandmarkIdSet segment_landmark_id_set;
    unsigned int num_segment_landmarks = 0;
    LOG(INFO) << "\tBuilding the landmark set for the segment";
    for (const pose_graph::VertexId& vertex_id : posegraph_partitioning_[i]) {
      for (const vi_map::Landmark landmark :
           map.getVertex(vertex_id).getLandmarks()) {
        ++num_segment_landmarks;
        if (map.getLandmark(landmark.id()).getQuality() ==
            vi_map::Landmark::Quality::kGood) {
          segment_landmark_id_set.insert(landmark.id());
        }
      }
    }

    unsigned int desired_num_landmarks = retain_ratio * num_segment_landmarks;

    // Time limit of the sampling process of a single map partition.
    const unsigned int kSegmentTimeLimitSeconds = 8;

    LOG(INFO) << "\tSampling out of " << segment_landmark_id_set.size()
              << " landmarks, desired: " << desired_num_landmarks;
    if (segment_landmark_id_set.size() > desired_num_landmarks) {
      vi_map::LandmarkIdSet segment_summary_landmark_ids;

      timing::Timer sampling_timer(
          "GraphPartitionSampler: " +
          std::to_string(posegraph_partitioning_.size()) +
          "partitions_sampling_timer");
      sampler_->sampleMapSegment(
          map, desired_num_landmarks, kSegmentTimeLimitSeconds,
          segment_landmark_id_set, posegraph_partitioning_[i],
          &segment_summary_landmark_ids);
      sampling_timer.Stop();

      LOG(INFO) << "\t" << segment_summary_landmark_ids.size()
                << " landmarks inserted from this segment.";
      summary_landmark_ids->insert(
          segment_summary_landmark_ids.begin(),
          segment_summary_landmark_ids.end());
    } else {
      LOG(WARNING) << "Landmark quality filtering left only "
                   << segment_landmark_id_set.size()
                   << " landmarks, less than " << desired_num_landmarks
                   << " landmarks desired. Summarization is not needed.";
      summary_landmark_ids->insert(segment_landmark_id_set.begin(),
                                         segment_landmark_id_set.end());
    }

    if (visualizer_) {
      partition_landmarks_[i].insert(segment_landmark_id_set.begin(),
                                     segment_landmark_id_set.end());

      const bool kGloballySelectedLandmarks = false;
      visualizer_->plotLandmarks(map, i, segment_landmark_id_set,
                                 posegraph_partitioning_, partition_landmarks_,
                                 kGloballySelectedLandmarks);
    }
  }

  if (max_partitioned_summarization_fraction_ < 1.0 &&
      summary_landmark_ids->size() > total_desired_num_landmarks) {
    LOG(WARNING) << "Global optimization -- from "
                 << summary_landmark_ids->size() << " to "
                 << total_desired_num_landmarks << " landmarks.";
    if (visualizer_) {
      const int kMarkAllVertices = -1;
      visualizer_->plotSegment(map, posegraph_partitioning_, kMarkAllVertices);
    }

    vi_map::LandmarkIdSet partitioned_summary_landmarks(
        summary_landmark_ids->begin(), summary_landmark_ids->end());

    pose_graph::VertexIdList all_vertex_ids;
    map.getAllVertexIds(&all_vertex_ids);

    // Time limit of the sampling process of a final, global sampling stage.
    const unsigned int kGlobalTimeLimitSeconds = 600;
    sampler_->sampleMapSegment(map, total_desired_num_landmarks,
                               kGlobalTimeLimitSeconds,
                               partitioned_summary_landmarks, all_vertex_ids,
                               summary_landmark_ids);
  }
  if (visualizer_) {
    const bool kGloballySelectedLandmarks = true;
    visualizer_->plotLandmarks(map, -1, *summary_landmark_ids,
                               posegraph_partitioning_, partition_landmarks_,
                               kGloballySelectedLandmarks);
  }
  LOG(INFO) << "Partitioned graph sampling done, landmarks left: "
            << summary_landmark_ids->size();
}

void GraphPartitionSampler::instantiateVisualizer() {
  visualizer_.reset(
      new map_sparsification_visualization::MapSparsificationVisualizer());
}

}  // namespace map_sparsification
