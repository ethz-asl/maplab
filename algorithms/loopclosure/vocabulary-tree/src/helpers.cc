#include <gflags/gflags.h>

#include "vocabulary-tree/helpers.h"

DEFINE_double(
    lc_kdtree_accelerator_eps, 0.1,
    "Epsilon for the NN search inside "
    "the kd-tree based search accelerators.");
