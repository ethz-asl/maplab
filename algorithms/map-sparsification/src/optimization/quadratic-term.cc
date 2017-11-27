#include "map-sparsification/optimization/quadratic-term.h"

namespace map_sparsification {

void getQuadraticCostCoobservanceSparseMatrix(
    const vi_map::VIMap& map,
    const std::unordered_map<vi_map::LandmarkId, unsigned int>&
        landmark_index_map,
    bool use_reference_setup,
    Eigen::SparseMatrix<double>* average_distance_matrix) {
  CHECK_NOTNULL(average_distance_matrix);

  typedef Eigen::Triplet<double> DoubleTriplet;
  typedef Eigen::Triplet<size_t> SizeTTriplet;

  // Two triplet lists to experiment with the average image plane distance
  // for a pair of landmarks.
  std::vector<DoubleTriplet> distance_triplet_list;
  std::vector<SizeTTriplet> common_frames_triplet_list;

  unsigned int vertex_index = 0;

  static constexpr double kMaxPixelDistanceBetweenKeypoints = 100.0;

  pose_graph::VertexIdList all_vertex_ids;
  map.getAllVertexIds(&all_vertex_ids);
  for (const pose_graph::VertexId& vertex_id : all_vertex_ids) {
    vi_map::LandmarkIdList observed_landmarks;
    const vi_map::Vertex& vertex = map.getVertex(vertex_id);

    const unsigned int kFrameNumber = 0;
    vertex.getFrameObservedLandmarkIds(kFrameNumber, &observed_landmarks);

    for (size_t i = 0; i < observed_landmarks.size(); ++i) {
      if (observed_landmarks[i].isValid()) {
        size_t index_i;
        if (!getLandmarkIndexForLandmarkId(
                observed_landmarks[i], landmark_index_map, &index_i)) {
          // Not in the landmark index provided, most probably we're performing
          // the partitioned summarization.
          continue;
        }

        // Verify each landmark pair in this frame.
        for (size_t j = i + 1; j < observed_landmarks.size(); ++j) {
          if (observed_landmarks[j].isValid()) {
            size_t index_j;
            if (!getLandmarkIndexForLandmarkId(
                    observed_landmarks[j], landmark_index_map, &index_j)) {
              // Not in the landmark index provided, most probably we're
              // performing the partitioned summarization.
              continue;
            }

            if (use_reference_setup) {
              // It's enough that the given landmarks were commonly observed
              // to introduce a unit weight in the Q matrix.
              distance_triplet_list.push_back(
                  DoubleTriplet(index_i, index_j, 1.0));
              common_frames_triplet_list.push_back(
                  SizeTTriplet(index_i, index_j, 1));
            } else {
              double norm = (vertex.getVisualFrame(kFrameNumber)
                                 .getKeypointMeasurement(i) -
                             vertex.getVisualFrame(kFrameNumber)
                                 .getKeypointMeasurement(j))
                                .norm();

              if (norm < kMaxPixelDistanceBetweenKeypoints) {
                const double weight =
                    (kMaxPixelDistanceBetweenKeypoints - norm) /
                    kMaxPixelDistanceBetweenKeypoints;

                distance_triplet_list.push_back(
                    DoubleTriplet(index_i, index_j, weight));
              }
              common_frames_triplet_list.push_back(
                  SizeTTriplet(index_i, index_j, 1));
            }
          }
        }
      }
    }
    ++vertex_index;
  }
  const size_t num_landmarks = landmark_index_map.size();

  Eigen::SparseMatrix<double> distance_matrix(num_landmarks, num_landmarks);
  Eigen::SparseMatrix<double> common_frames_count_matrix(
      num_landmarks, num_landmarks);

  distance_matrix.setFromTriplets(
      distance_triplet_list.begin(), distance_triplet_list.end());
  common_frames_count_matrix.setFromTriplets(
      common_frames_triplet_list.begin(), common_frames_triplet_list.end());
  if (use_reference_setup) {
    *average_distance_matrix = distance_matrix;
  } else {
    *average_distance_matrix =
        distance_matrix.cwiseQuotient(common_frames_count_matrix);
  }
}

bool getLandmarkIndexForLandmarkId(
    const vi_map::LandmarkId& landmark_id,
    const std::unordered_map<vi_map::LandmarkId, unsigned int>&
        landmark_index_map,
    size_t* landmark_index) {
  CHECK_NOTNULL(landmark_index);

  typedef std::unordered_map<vi_map::LandmarkId, unsigned int> LandmarkIndexMap;
  LandmarkIndexMap::const_iterator it = landmark_index_map.find(landmark_id);
  if (it == landmark_index_map.end()) {
    // Not in the landmark index provided, most probably we're
    // performing the partitioned summarization.
    return false;
  }
  *landmark_index = it->second;
  return true;
}

}  // namespace map_sparsification
