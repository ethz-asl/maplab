#ifndef VI_MAP_HELPERS_VI_MAP_PARTITIONER_H_
#define VI_MAP_HELPERS_VI_MAP_PARTITIONER_H_

#include <string>
#include <unordered_map>
#include <vector>

#include <Eigen/Sparse>
#include <vi-map/vi-map.h>

namespace vi_map_helpers {

class VIMapPartitioner {
 public:
  void exportGraphToMetis(const vi_map::VIMap& map) const;

  void readMetisOutput(
      const size_t num_partitions, const vi_map::VIMap& map,
      std::vector<pose_graph::VertexIdList>* partitioning);

  // Partitions the vertices of the VIMap based on the coobserved landmarks
  // graph.
  void partitionMapWithMetis(
      const vi_map::VIMap& map, const unsigned int num_partitions,
      std::vector<pose_graph::VertexIdList>* partitioning) const;

 private:
  struct GraphEdge {
    size_t vertex_index;
    size_t score;
  };
  typedef std::vector<GraphEdge> GraphEdgeVector;
  typedef Eigen::Triplet<int32_t> EdgeTriplet;

  // This method finds coobserver edges between posegraph vertices. The edges
  // are weighted according to the number of landmarks coobserved by the two
  // vertices.
  //
  // It supports vertex indices as provided by assignAndGetVertexIndices. The
  // output of the method is also ordered according to the vertex indexing.
  //
  // min_number_of_common_landmarks denotes the minimum number of coobserved
  // landmarks to create an edge.
  void getCoobserverEdges(
      const vi_map::VIMap& map,
      const std::unordered_map<pose_graph::VertexId, size_t>& vertex_indices,
      const unsigned int min_number_of_common_landmarks,
      unsigned int* total_num_edges, std::vector<GraphEdgeVector>* edges) const;

  // This method provides a mapping between VIMap vertex IDs and vertex indices
  // that are used by METIS interface.
  void assignAndGetVertexIndices(
      const vi_map::VIMap& map,
      std::unordered_map<pose_graph::VertexId, size_t>* vertex_indices) const;

  void partitionGraphFromTriplets(
      const std::vector<EdgeTriplet>& edge_triplets,
      const size_t num_graph_vertices, const size_t num_partitions,
      bool require_contiguous_partitions,
      std::vector<int32_t>* partition_indices) const;

  static const std::string kFileName;
  static constexpr size_t kMinNumberOfCoobservedLandmarks = 5;
};

}  // namespace vi_map_helpers
#endif  // VI_MAP_HELPERS_VI_MAP_PARTITIONER_H_
