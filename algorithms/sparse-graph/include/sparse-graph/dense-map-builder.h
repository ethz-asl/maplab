#ifndef SPARSE_GRAPH_DENSE_MAP_BUILDER_H_
#define SPARSE_GRAPH_DENSE_MAP_BUILDER_H_

#include <vi-map/vi-map.h>

namespace spg {

class DenseMapBuilder {
 public:
  explicit DenseMapBuilder(const vi_map::VIMap* map);
  void buildMapFromVertices(const pose_graph::VertexIdList& vertices);

 private:
  std::vector<int64_t> getTimestampsFromVertices(
      const pose_graph::VertexIdList& vertices);
  vi_map::MissionIdList getMissionIdsFromVertices(
      const pose_graph::VertexIdList& vertices);

  const vi_map::VIMap* map_;

}  // namespace spg

#endif  // SPARSE_GRAPH_DENSE_MAP_BUILDER_H_
