#include "dense-mapping/dense-mapping-gflags.h"

#include <gflags/gflags.h>
#include <glog/logging.h>

// CANDIDATE SEARCH
DEFINE_string(
    dm_resource_types, "16,17,21,24",
    "Defines the resource types that are used to derive the dense mapping "
    "constraints. Provide a CSV string with the resource type numbers.");

DEFINE_bool(
    dm_candidate_search_enable_intra_mission_consecutive, true,
    "If enabled, the algorithm will try to find dense mapping constraints "
    "between consecuive dense data along the pose graph, within a single "
    "mission.");
DEFINE_double(
    dm_candidate_search_consecutive_max_delta_time_s, 20.0,
    "Maximum time between dense mapping constraints along the pose graph. The "
    "search will try to find corresponding dense data pairs at a lower "
    "interval.");
DEFINE_double(
    dm_candidate_search_consecutive_max_delta_position_m, 5.0,
    "Maximum delta position between dense mapping constraints along the pose "
    "graph. The search will try to find corresponding dense data pairs that "
    "are closer.");
DEFINE_double(
    dm_candidate_search_consecutive_max_delta_rotation_deg, 180.0,
    "Maximum delta rotation between dense mapping constraints along the pose "
    "graph. The search will try to find corresponding dense data pairs with "
    "less rotation.");

DEFINE_bool(
    dm_candidate_search_enable_intra_mission_proximity, true,
    "If enabled, the algorithm will try to find dense mapping constraints "
    "between nearby/overlapping dense data, within a single mission.");
DEFINE_bool(
    dm_candidate_search_enable_intra_mission_global, true,
    "If enabled, the algorithm will try to find dense mapping constraints "
    "between any dense data based on a global place recognition algorithm, "
    "within a single mission.");

DEFINE_double(
    dm_candidate_search_proximity_max_delta_position_m, 2.0,
    "Maximum delta position between dense mapping constraints based on nearby "
    "vertices of the pose graph. The search will try to find corresponding "
    "dense data pairs that are closer.");
DEFINE_double(
    dm_candidate_search_proximity_max_delta_rotation_deg, 180.0,
    "Maximum delta rotation between dense mapping constraints based on nearby "
    "vertices of the pose graph. The search will try to find corresponding "
    "dense data pairs with less rotation.");
DEFINE_double(
    dm_candidate_search_proximity_min_distance_along_graph_m, 1.0,
    "Minimum distance along graph between dense mapping constraints based on "
    "nearby vertices of the pose graph. The search will try to find "
    "corresponding dense data pairs that are not too close along the "
    "pose-graph. This will therefore only affect intra-mission candidates.");
DEFINE_int32(
    dm_candidate_search_proximity_take_closest_n_candidates, 3,
    "If enabled (> 0), the proximity search will only take the N closest "
    "candidates.");

DEFINE_bool(
    dm_candidate_search_enable_inter_mission_proximity, true,
    "If enabled, the algorithm will try to find dense mapping constraints "
    "between nearby/overlapping dense data, across missions.");
DEFINE_bool(
    dm_candidate_search_enable_inter_mission_global, true,
    "If enabled, the algorithm will try to find dense mapping constraints "
    "between any dense data based on a global place recognition algorithm, "
    "across missions.");

// CANDIDATE SELECTION
DEFINE_bool(
    dm_candidate_selection_recompute_all_constraints, false,
    "If enabled, the algorithm will recompute the alignment between two "
    "candidates even if a valid constraint is already present in the map.");
DEFINE_bool(
    dm_candidate_selection_recompute_invalid_constraints, true,
    "If enabled, the algorithm will recompute the alignment between two "
    "candidates even if the constraint has been invalidated (e.g. by the "
    "optimization).");
DEFINE_double(
    dm_candidate_selection_min_switch_variable_value, 0.75,
    "Threshold for the switch variable to classify a constraint as valid.");
DEFINE_int32(
    dm_candidate_selection_max_number_of_candidates, 0,
    "Limits the number of candidates according to the filter strategy. "
    "Zero means no limit.");
DEFINE_string(
    dm_candidate_selection_filter_strategy, "random",
    "Filter strategy [random] for the candidate selection process.");
DEFINE_double(
    dm_candidate_selection_prioritize_recent_candidates, 0.0,
    "Defines the percentage of fixed recent candidates [0,1].");

// ALIGNMENTS
DEFINE_double(
    dm_candidate_alignment_max_delta_position_to_initial_guess_m, 0.5,
    "Maximum translation deviation of the alignment from the initial "
    "guess.");
DEFINE_double(
    dm_candidate_alignment_max_delta_rotation_to_initial_guess_deg, 10,
    "Maximum angular deviation of the alignment from the initial guess.");
DEFINE_string(
    dm_candidate_alignment_type, "PclGIcp",
    "Alignment type that is used for the pointcloud registration");
DEFINE_bool(
    dm_candidate_alignment_use_incremental_submab_alignment, true,
    "If enabled, an incremental map is built and alignment is performed "
    "against it");

// CONSTRAINTS
DEFINE_double(
    dm_constraint_switch_variable_value, 1.0,
    "Initial value of switch variable of loop closure edge that is used to "
    "enforce the dense mapping constraint.");
DEFINE_double(
    dm_constraint_switch_variable_sigma, 1e-3,
    "Sigma of switch variable of loop closure edge that is used to "
    "enforce the dense mapping constraint.");

// VISUALIZATION
DEFINE_bool(
    dm_visualize_incremental_submap, false,
    "If enabled, the incremental map is visualized");
