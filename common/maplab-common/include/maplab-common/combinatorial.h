#ifndef MAPLAB_COMMON_COMBINATORIAL_H_
#define MAPLAB_COMMON_COMBINATORIAL_H_

#include <vector>

#include <Eigen/Core>
#include <aslam/common/memory.h>

namespace common {

// Returns the power set with num_elements elements.
// E.g. if num_elements == 3, then this function returns
// [(0,0,0), (1,0,0), (0,1,0), (0,0,1), (1,1,0), (0,1,1)
//  (1,0,1), (1,1,1)]
void getAllBinaryCombinations(
    size_t num_elements,
    Aligned<std::vector, Eigen::VectorXi>* list_of_combinations);

}  // namespace common

#endif  // MAPLAB_COMMON_COMBINATORIAL_H_
