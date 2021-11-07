#ifndef SEMANTIFY_PLUGIN_HUNGARIAN_ALGORITHM_MANAGER_H_
#define SEMANTIFY_PLUGIN_HUNGARIAN_ALGORITHM_MANAGER_H_

#include <map>
#include <memory>
#include <vector>

#include <Eigen/Dense>

#include "semantify-plugin/hungarian-algorithm.h"

namespace semantify {

class HungaraianAlgorithmManager {
public:
  HungaraianAlgorithmManager(
      std::map<int, Eigen::VectorXf> track_id_to_medoid_descriptors);
  void solve();
  std::vector<int> getAssignments();
private:
  void build_cost_matrix(const std::map<int, Eigen::VectorXf>& track_id_to_medoid_descriptors);

  std::unique_ptr<algorithm::HungarianAlgorithm> solver_;
  std::vector<std::vector<double>> cost_matrix_;
  std::vector<int> assignments_;
};
}  // namespace semantify
#endif  // SEMANTIFY_PLUGIN_HUNGARIAN_ALGORITHM_MANAGER_H_