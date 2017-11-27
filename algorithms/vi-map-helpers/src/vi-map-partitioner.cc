#include "vi-map-helpers/vi-map-partitioner.h"

#include <fstream>  // NOLINT
#include <vector>

#include <maplab-common/file-logger.h>
#include <maplab-common/progress-bar.h>
#include <metis.h>
#include <vi-map-helpers/vi-map-queries.h>
#include <vi-map/vi-map.h>

namespace vi_map_helpers {

const std::string VIMapPartitioner::kFileName = "metis_graph";

void VIMapPartitioner::partitionMapWithMetis(
    const vi_map::VIMap& map, const unsigned int num_partitions,
    std::vector<pose_graph::VertexIdList>* partitioning) const {
  CHECK_NOTNULL(partitioning)->clear();
  partitioning->resize(num_partitions);

  // An unordered map for quick index retrieval.
  std::unordered_map<pose_graph::VertexId, size_t> vertex_indices;
  assignAndGetVertexIndices(map, &vertex_indices);
  const unsigned int num_vertices = vertex_indices.size();

  std::vector<GraphEdgeVector> edges;
  unsigned int total_num_edges;
  getCoobserverEdges(
      map, vertex_indices, kMinNumberOfCoobservedLandmarks, &total_num_edges,
      &edges);

  typedef Eigen::Triplet<idx_t> EdgeTriplet;
  std::vector<EdgeTriplet> edge_triplets;
  // Triplets will store each edge twice (= in both directions).
  edge_triplets.reserve(2 * total_num_edges);
  for (unsigned int i = 0; i < edges.size(); ++i) {
    for (const GraphEdge& edge : edges[i]) {
      edge_triplets.push_back(EdgeTriplet(i, edge.vertex_index, edge.score));
    }
  }

  std::vector<idx_t> part;
  static constexpr bool kRequireContiguous = true;
  partitionGraphFromTriplets(
      edge_triplets, num_vertices, num_partitions, kRequireContiguous, &part);

  pose_graph::VertexIdList all_vertex_ids;
  map.getAllVertexIds(&all_vertex_ids);
  for (const pose_graph::VertexId& vertex_id : all_vertex_ids) {
    std::unordered_map<pose_graph::VertexId, size_t>::const_iterator it =
        vertex_indices.find(vertex_id);
    CHECK(it != vertex_indices.end());
    CHECK_LT(it->second, part.size());
    (*partitioning)[part[it->second]].push_back(vertex_id);
  }
}

void VIMapPartitioner::partitionGraphFromTriplets(
    const std::vector<EdgeTriplet>& edge_triplets,
    const size_t num_graph_vertices, const size_t num_partitions,
    const bool require_contiguous_partitions,
    std::vector<int32_t>* partition_indices) const {
  CHECK_NOTNULL(partition_indices)->clear();
  partition_indices->resize(num_graph_vertices);

  Eigen::SparseMatrix<idx_t> edge_matrix(
      num_graph_vertices, num_graph_vertices);
  edge_matrix.setFromTriplets(edge_triplets.begin(), edge_triplets.end());
  edge_matrix.makeCompressed();

  // idx_t and real_t are types defined by METIS in metis.h header file.
  idx_t nvtxs = num_graph_vertices;
  // Single constraint per vertex.
  idx_t ncon = 1;
  idx_t* vwgt = NULL;
  idx_t* vsize = NULL;
  idx_t nparts = num_partitions;
  real_t* tpwgts = NULL;
  real_t* ubvec = NULL;
  idx_t objval;

  idx_t options[METIS_NOPTIONS];
  METIS_SetDefaultOptions(options);

  // Force contiguous partitions if possible.
  options[METIS_OPTION_CONTIG] = (require_contiguous_partitions ? 1u : 0u);

  // Default options of the command line METIS.
  // For details see:
  // http://glaros.dtc.umn.edu/gkhome/fetch/sw/metis/manual.pdf
  //
  // K-way graph partitioning that supports contiguous partitions constraint.
  options[METIS_OPTION_PTYPE] = METIS_PTYPE_KWAY;
  // Objective: edge-cut minimization.
  options[METIS_OPTION_OBJTYPE] = METIS_OBJTYPE_CUT;
  // Coarsening method: sorted heavy-edge.
  options[METIS_OPTION_CTYPE] = METIS_CTYPE_SHEM;
  // Refinement method: Greedy-based cut and volume refinement.
  options[METIS_OPTION_RTYPE] = METIS_RTYPE_GREEDY;
  // Initial partitioning method: not documented MetisRB method, used by
  // default by command-line tool.
  options[METIS_OPTION_IPTYPE] = METIS_IPTYPE_RANDOM;
  // Partitioning will try to minimize the maximum degree of subdomain graph.
  options[METIS_OPTION_MINCONN] = 0u;
  // Coarsening will use 2-hop matching if needed.
  options[METIS_OPTION_NO2HOP] = 0u;
  // Indexing starting from 0 (C-style).
  options[METIS_OPTION_NUMBERING] = 0u;
  // Output timing and initial partitioning information.
  options[METIS_OPTION_DBGLVL] = METIS_DBG_TIME | METIS_DBG_IPART;

  int status = METIS_PartGraphKway(
      &nvtxs, &ncon, edge_matrix.outerIndexPtr(), edge_matrix.innerIndexPtr(),
      vwgt, vsize, edge_matrix.valuePtr(), &nparts, tpwgts, ubvec, options,
      &objval, &partition_indices->front());

  switch (status) {
    case METIS_OK:
      LOG(INFO) << "METIS partitioning finished.";
      break;
    case METIS_ERROR_INPUT:
      LOG(FATAL) << "Wrong METIS input format.";
      break;
    case METIS_ERROR_MEMORY:
      LOG(FATAL) << "METIS could not allocate memory.";
      break;
    case METIS_ERROR:
      if (require_contiguous_partitions) {
        LOG(WARNING) << "Experienced a METIS error, will omit the contiguous "
                     << "graph constraint and try again.";
        static constexpr bool kRequireContiguousPartitions = false;
        partitionGraphFromTriplets(
            edge_triplets, num_graph_vertices, num_partitions,
            kRequireContiguousPartitions, partition_indices);
      } else {
        LOG(FATAL) << "METIS error.";
      }
      break;
    default:
      LOG(FATAL) << "Unknown METIS return code.";
      break;
  }
}

void VIMapPartitioner::getCoobserverEdges(
    const vi_map::VIMap& map,
    const std::unordered_map<pose_graph::VertexId, size_t>& vertex_indices,
    const unsigned int min_number_of_common_landmarks,
    unsigned int* total_num_edges, std::vector<GraphEdgeVector>* edges) const {
  CHECK_NOTNULL(edges)->clear();
  *CHECK_NOTNULL(total_num_edges) = 0u;
  CHECK_EQ(vertex_indices.size(), map.numVertices());

  const size_t num_vertices = vertex_indices.size();
  edges->resize(num_vertices);

  typedef std::pair<pose_graph::VertexId, size_t> VertexIndexPair;

  vi_map_helpers::VIMapQueries vi_map_queries(map);
  unsigned int index = 0u;
  common::ProgressBar progress_bar(num_vertices);
  LOG(INFO) << "Partitioning the graph...";
  for (const VertexIndexPair& vertex_index : vertex_indices) {
    VLOG(5) << "Processing vertex " << index++ << "/" << (num_vertices - 1);

    vi_map_helpers::VIMapQueries::VertexCommonLandmarksCountVector
        coobserver_vertex_ids;

    vi_map_queries.getVerticesWithCommonLandmarks(
        vertex_index.first, min_number_of_common_landmarks,
        &coobserver_vertex_ids);

    for (const vi_map_helpers::VIMapQueries::VertexCommonLandmarksCount&
             coobserver_vertex : coobserver_vertex_ids) {
      std::unordered_map<pose_graph::VertexId, size_t>::const_iterator it =
          vertex_indices.find(coobserver_vertex.vertex_id);
      CHECK(it != vertex_indices.end());
      unsigned int coobserver_index = it->second;
      CHECK_LT(coobserver_index, edges->size());

      GraphEdge edge_to_coobserver;
      edge_to_coobserver.vertex_index = coobserver_index;
      edge_to_coobserver.score = coobserver_vertex.in_common;
      (*edges)[vertex_index.second].push_back(edge_to_coobserver);

      GraphEdge edge_from_coobserver;
      edge_from_coobserver.vertex_index = vertex_index.second;
      edge_from_coobserver.score = coobserver_vertex.in_common;
      (*edges)[coobserver_index].push_back(edge_from_coobserver);

      ++(*total_num_edges);
    }
    progress_bar.increment();
  }
}

void VIMapPartitioner::assignAndGetVertexIndices(
    const vi_map::VIMap& map,
    std::unordered_map<pose_graph::VertexId, size_t>* vertex_indices) const {
  CHECK_NOTNULL(vertex_indices)->clear();

  pose_graph::VertexIdList all_vertex_ids;
  map.getAllVertexIds(&all_vertex_ids);
  const unsigned int num_vertices = all_vertex_ids.size();

  vertex_indices->reserve(num_vertices);
  for (unsigned int i = 0; i < num_vertices; ++i) {
    vertex_indices->emplace(all_vertex_ids[i], i);
  }
}

void VIMapPartitioner::exportGraphToMetis(const vi_map::VIMap& map) const {
  // An unordered map for quick index retrieval.
  std::unordered_map<pose_graph::VertexId, size_t> vertex_indices;
  assignAndGetVertexIndices(map, &vertex_indices);
  const unsigned int num_vertices = vertex_indices.size();

  std::vector<GraphEdgeVector> edges;
  unsigned int total_num_edges;
  getCoobserverEdges(
      map, vertex_indices, kMinNumberOfCoobservedLandmarks, &total_num_edges,
      &edges);

  LOG(INFO) << "Exporting graph data to a file...";
  common::FileLogger metis_graph_export(kFileName);
  CHECK(metis_graph_export.isOpen())
      << "Couldn't create the output METIS partitioning file.";
  // According to METIS format:
  // * number of vertices
  // * number of edges
  // * 001 - weights only on edges
  metis_graph_export << num_vertices << " " << total_num_edges << " 001"
                     << std::endl;
  for (unsigned int i = 0; i < num_vertices; ++i) {
    for (const GraphEdge& edge : edges[i]) {
      // METIS requires vertex numbering to start from 1.
      metis_graph_export << (edge.vertex_index + 1) << " " << edge.score << " ";
    }
    metis_graph_export << std::endl;
  }
  metis_graph_export.flushBuffer();
}

void VIMapPartitioner::readMetisOutput(
    const size_t num_partitions, const vi_map::VIMap& map,
    std::vector<pose_graph::VertexIdList>* partitioning) {
  CHECK_NOTNULL(partitioning)->clear();

  std::ifstream partitioning_file;
  // Default METIS output filename: input_file.part.num_of_partitions
  partitioning_file.open(kFileName + ".part." + std::to_string(num_partitions));
  CHECK(partitioning_file.is_open())
      << "Couldn't open the input METIS partitioning file, expected filename: "
      << kFileName + ".part." + std::to_string(num_partitions);

  pose_graph::VertexIdList all_vertex_ids;
  map.getAllVertexIds(&all_vertex_ids);
  const unsigned int num_vertices = all_vertex_ids.size();

  for (unsigned int i = 0; i < num_vertices; ++i) {
    unsigned int cluster_index;
    partitioning_file >> cluster_index;

    if (cluster_index >= partitioning->size()) {
      partitioning->resize(cluster_index + 1);
    }

    (*partitioning)[cluster_index].push_back(all_vertex_ids[i]);
  }
}

}  // namespace vi_map_helpers
