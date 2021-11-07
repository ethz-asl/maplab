#include "semantify-plugin/hungarian-algorithm-manager.h"

#include <iostream>
#include <string>
#include <limits>

#include <glog/logging.h>

namespace semantify {
HungaraianAlgorithmManager::HungaraianAlgorithmManager (
  std::map<int, Eigen::VectorXf> track_id_to_medoid_descriptors) {

  build_cost_matrix(track_id_to_medoid_descriptors);

}

void HungaraianAlgorithmManager::solve() {
  assignments_.clear();
  double cost = solver_->Solve(cost_matrix_, assignments_);
  for (size_t i = 0; i < cost_matrix_.size(); i++) {
    LOG(INFO)<< i << " -> " << assignments_[i];
  }
}

std::vector<int> HungaraianAlgorithmManager::getAssignments() {
  return assignments_;
}

// private
void HungaraianAlgorithmManager::build_cost_matrix (
  const std::map<int, Eigen::VectorXf>& track_id_to_medoid_descriptors) {
  CHECK_GT(track_id_to_medoid_descriptors.size(),0u);
  cost_matrix_.clear();
  for(const auto& pair_i : track_id_to_medoid_descriptors) {
    std::vector<double> i_to_j_cost;
    for(const auto& pair_j : track_id_to_medoid_descriptors) {
      if (pair_i.first == pair_j.first) {
        i_to_j_cost.push_back(std::numeric_limits<double>::max());
      } else {
        i_to_j_cost.push_back(static_cast<double>((pair_i.second-pair_j.second).norm()));
      }
    }
    cost_matrix_.push_back(i_to_j_cost);
  }
  
  // please use "-std=c++11" for this initialization of vector.
  // vector< vector<double> > costMatrix = { { 10, 19, 8, 15, 0 }, 
  //                     { 10, 18, 7, 17, 0 }, 
  //                     { 13, 16, 9, 14, 0 }, 
  //                     { 12, 19, 8, 18, 0 } };

  // HungarianAlgorithm HungAlgo;
  // vector<int> assignment;

  // double cost = HungAlgo.Solve(costMatrix, assignment);

  // for (unsigned int x = 0; x < costMatrix.size(); x++)
  //   std::cout << x << "," << assignment[x] << "\t";

  // std::cout << "\ncost: " << cost << std::endl;


}

} // namespace semantify