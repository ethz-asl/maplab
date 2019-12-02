#include "map-optimization/solver-options.h"

DEFINE_int32(ba_num_iterations, 30, "Max. number of iterations.");
DEFINE_bool(
    ba_enable_signal_handler, true,
    "If enabled, the optimization will register a signal handler that allows "
    "the user to abort the optimization after the next iterations with "
    "Ctrl-C.");
DEFINE_bool(ba_use_jacobi_scaling, true, "Use jacobin scaling.");
DEFINE_bool(ba_use_cgnr_linear_solver, false, "Use CGNR linear solver?");
