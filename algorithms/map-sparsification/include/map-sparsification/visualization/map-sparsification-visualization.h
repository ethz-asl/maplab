#ifndef VISUALIZATION_MAP_SPARSIFICATION_VISUALIZATION_H_
#define VISUALIZATION_MAP_SPARSIFICATION_VISUALIZATION_H_

#include <string>
#include <vector>

#include <posegraph/unique-id.h>
#include <vi-map/vi-map.h>

namespace map_sparsification_visualization {

class MapSparsificationVisualizer {
 public:
  MapSparsificationVisualizer()
      : segment_vertices_topic_("segment_vertices"),
        landmarks_topic_("landmarks") {}
  virtual ~MapSparsificationVisualizer();

  // When segment_index is equal to -1, mark all the vertices as selected.
  void plotSegment(
      const vi_map::VIMap& map,
      const std::vector<pose_graph::VertexIdList>& posegraph_partitioning,
      int segment_index) const;
  // Plots landmarks to visualize the partitioning and summarization process.
  //   * map - VI map object
  //   * segment_index - the segment the landmarks belong to, pass -1 for
  //                     landmarks that don't belong to any partition (either
  //                     in case of final global optimization or when plotting
  //                     the initial state)
  //   * selected_landmark_ids - list of selected store landmark ids
  //   * are_landmarks_globally_selected - pass 'true' if the result of the
  // final
  //                                       global optimization should be plotted
  void plotLandmarks(
      const vi_map::VIMap& map, int segment_index,
      const vi_map::LandmarkIdSet& selected_landmark_ids,
      const std::vector<pose_graph::VertexIdList>& posegraph_partitioning,
      const std::vector<vi_map::LandmarkIdSet>& partition_landmarks,
      bool are_landmarks_globally_selected) const;

 private:
  const std::string segment_vertices_topic_;
  const std::string landmarks_topic_;
};
}  // namespace map_sparsification_visualization

#endif  // VISUALIZATION_MAP_SPARSIFICATION_VISUALIZATION_H_
