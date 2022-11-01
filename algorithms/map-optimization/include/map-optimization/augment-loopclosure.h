#ifndef MAP_OPTIMIZATION_AUGMENT_LOOPCLOSURE_H_
#define MAP_OPTIMIZATION_AUGMENT_LOOPCLOSURE_H_

#include <map-optimization/optimization-problem.h>

namespace map_optimization {
int augmentOptimizationProblemWithLoopclosureEdges(
    OptimizationProblem* problem);

int numLoopclosureEdges(const vi_map::VIMap& map);

}  // namespace map_optimization
#endif  // MAP_OPTIMIZATION_AUGMENT_LOOPCLOSURE_H_
