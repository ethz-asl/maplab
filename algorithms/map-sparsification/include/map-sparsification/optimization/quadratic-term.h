#ifndef MAP_SPARSIFICATION_OPTIMIZATION_QUADRATIC_TERM_H_
#define MAP_SPARSIFICATION_OPTIMIZATION_QUADRATIC_TERM_H_

#include <Eigen/Sparse>
#include <vi-map/unique-id.h>
#include <vi-map/vi-map.h>

namespace map_sparsification {

// use_reference_setup set to true will construct the matrix exactly according
// to H. S. Park et al. "3D Point Cloud Reduction using Mixed-integer Quadratic
// Programming".
void getQuadraticCostCoobservanceSparseMatrix(
    const vi_map::VIMap& map,
    const std::unordered_map<vi_map::LandmarkId, unsigned int>&
        landmark_index_map,
    bool use_reference_setup,
    Eigen::SparseMatrix<double>* average_distance_matrix);

bool getLandmarkIndexForLandmarkId(
    const vi_map::LandmarkId& landmark_id,
    const std::unordered_map<vi_map::LandmarkId, unsigned int>&
        landmark_index_map,
    size_t* landmark_index);

}  // namespace map_sparsification
#endif  // MAP_SPARSIFICATION_OPTIMIZATION_QUADRATIC_TERM_H_
