#ifndef MAP_SPARSIFICATION_OPTIMIZATION_LPREC_WRAPPER_H_
#define MAP_SPARSIFICATION_OPTIMIZATION_LPREC_WRAPPER_H_

#include <memory>

namespace map_sparsification {

#include <lp_solve/lp_lib.h>

struct LprecWrapper {
  lprec* lprec_ptr;
};

}  // namespace map_sparsification
#endif  // MAP_SPARSIFICATION_OPTIMIZATION_LPREC_WRAPPER_H_
