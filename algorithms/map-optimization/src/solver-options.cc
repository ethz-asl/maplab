#include "map-optimization/solver-options.h"

DEFINE_int32(ba_num_iterations, 30, "Max. number of iterations.");
DEFINE_int32(
    ba_max_time_seconds, 1e6,
    "Maximum amount of time for which the solver should run.");
DEFINE_bool(
    ba_enable_signal_handler, true,
    "If enabled, the optimization will register a signal handler that allows "
    "the user to abort the optimization after the next iterations with "
    "Ctrl-C.");
DEFINE_bool(ba_use_jacobi_scaling, true, "Use jacobin scaling.");
DEFINE_string(
    ba_linear_solver_type, "SPARSE_NORMAL_CHOLESKY",
    "Options: 'CGNR', 'SPARSE_NORMAL_CHOLESKY', 'SPARSE_SCHUR', "
    "'ITERATIVE_SCHUR'.");
DEFINE_bool(
    ba_use_nonmonotonic_steps, false,
    "If enabled, the objective function is allowed to get worse for a max "
    "number of steps define with --ba_max_consecutive_nonmonotonic_steps. "
    "The idea is that in the long run the optimization is able to 'jump over "
    "boulders' in an attempt to find a better solution. (Ceres default: "
    "false)");
DEFINE_int32(
    ba_max_consecutive_nonmonotonic_steps, 5,
    "Maximum number of steps where the cost is increasing. This only has an "
    "effect if --ba_use_nonmonotonic_steps is enabled. (Ceres default: 5)");
DEFINE_double(
    ba_initial_trust_region_radius, 1e5,
    "Initial radius of trust region. (Ceres default: 1e4)");
DEFINE_double(
    ba_max_trust_region_radius, 1e20,
    "Maximum radius of trust region. (Ceres default: 1e16)");
DEFINE_double(
    ba_min_trust_region_radius, 1e-32,
    "Minimum radius of trust region. (Ceres default: 1e-32)");
DEFINE_double(
    ba_function_tolerance, 1e-12,
    "The minimizer terminates if (new_cost - old_cost) < function_tolerance * "
    "old_cost. (Ceres default: 1e-6)");
DEFINE_double(
    ba_gradient_tolerance, 1e-10,
    "The minimizer terminates if: max_i |x - Project(Plus(x, -g(x))| < "
    "gradient_tolerance. (Ceres default: 1e-10)");
DEFINE_double(
    ba_parameter_tolerance, 1e-8,
    "The minimizer terminates if: |step|_2 <= parameter_tolerance * ( |x|_2 +  "
    "parameter_tolerance). (Ceres default: 1e-8)");
DEFINE_string(
    ba_sparse_linear_algebra_library_type, "SUITE_SPARSE",
    "Default: The highest available according to: SUITE_SPARSE > CX_SPARSE > "
    "EIGEN_SPARSE > NO_SPARSE");
