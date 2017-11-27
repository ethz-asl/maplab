#ifndef MAP_SPARSIFICATION_GRAPH_PARTITION_SAMPLER_H_
#define MAP_SPARSIFICATION_GRAPH_PARTITION_SAMPLER_H_

#include <memory>
#include <string>
#include <vector>

#include <maplab-common/macros.h>
#include <vi-map/unique-id.h>
#include <vi-map/vi-map.h>

#include "map-sparsification/sampler-base.h"

namespace map_sparsification_visualization {
class MapSparsificationVisualizer;
}

namespace map_sparsification {

class GraphPartitionSampler : public SamplerBase {
 public:
  MAPLAB_POINTER_TYPEDEFS(GraphPartitionSampler);

  explicit GraphPartitionSampler(map_sparsification::SamplerBase::Ptr sampler);
  virtual ~GraphPartitionSampler();

  void setMaxPartitionedSummarizationFraction(double fraction);

  virtual void sample(
      const vi_map::VIMap& map, unsigned int total_desired_num_landmarks,
      vi_map::LandmarkIdSet* summary_store_landmark_ids);

  virtual void sampleMapSegment(
      const vi_map::VIMap& map, unsigned int desired_num_landmarks,
      unsigned int /*time_limit_seconds*/,
      const vi_map::LandmarkIdSet& /*segment_store_landmark_id_set*/,
      const pose_graph::VertexIdList& /*segment_vertex_id_list*/,
      vi_map::LandmarkIdSet* summary_store_landmark_ids) {
    sample(map, desired_num_landmarks, summary_store_landmark_ids);
  }

  virtual std::string getTypeString() const {
    return sampler_->getTypeString();
  }

  void instantiateVisualizer();

 private:
  void partitionMapIfNecessary(const vi_map::VIMap& map);

  void plotSegment(const vi_map::VIMap& map, int segment_index);
  void plotLandmarks(
      const vi_map::VIMap& map, int segment_index,
      const vi_map::LandmarkIdSet& selected_landmark_ids,
      bool are_globally_selected);

  map_sparsification::SamplerBase::Ptr sampler_;
  std::vector<pose_graph::VertexIdList> posegraph_partitioning_;
  double max_partitioned_summarization_fraction_;

  std::vector<vi_map::LandmarkIdSet> partition_landmarks_;

  std::unique_ptr<map_sparsification_visualization::MapSparsificationVisualizer>
      visualizer_;

  static constexpr size_t kMaxNumLandmarkPerPartition = 5000u;
};

}  // namespace map_sparsification
#endif  // MAP_SPARSIFICATION_GRAPH_PARTITION_SAMPLER_H_
