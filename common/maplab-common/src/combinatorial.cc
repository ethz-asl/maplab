#include "maplab-common/combinatorial.h"

#include <Eigen/Core>
#include <glog/logging.h>

namespace common {

void getAllBinaryCombinations(
    size_t num_elements,
    Aligned<std::vector, Eigen::VectorXi>* list_of_combinations) {
  CHECK_NOTNULL(list_of_combinations)->clear();

  if (num_elements == 0u) {
    return;
  }

  const size_t num_combinations = 1 << num_elements;
  list_of_combinations->resize(
      num_combinations, Eigen::VectorXi::Zero(num_elements));

  for (size_t offset = 0; offset < num_elements; ++offset) {
    const size_t interval = 1 << offset;
    size_t combination_idx = 0u;
    int alternating_boolean = 0;
    while (combination_idx < num_combinations) {
      for (size_t interval_idx = 0u; interval_idx < interval; ++interval_idx) {
        (*list_of_combinations)[combination_idx](offset) = alternating_boolean;
        ++combination_idx;
      }
      alternating_boolean = (alternating_boolean + 1) % 2;
    }
  }
}

}  // namespace common
