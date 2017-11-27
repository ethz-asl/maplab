#include "map-sparsification/optimization/lp-solve-sparsification.h"

#include <limits>
#include <memory>
#include <vector>

#include <glog/logging.h>
#include <lp_solve/lp_lib.h>

#include "map-sparsification/optimization/lprec-wrapper.h"

namespace map_sparsification {

void LpSolveSparsification::sampleMapSegment(
    const vi_map::VIMap& map, unsigned int desired_num_landmarks,
    unsigned int /*time_limit_seconds*/,
    const vi_map::LandmarkIdSet& segment_landmark_id_set,
    const pose_graph::VertexIdList& segment_vertex_id_list,
    vi_map::LandmarkIdSet* summary_landmark_ids) {
  CHECK_NOTNULL(summary_landmark_ids)->clear();

  // Reset class members.
  landmark_ids_to_indices_.clear();
  num_variables_ = 0u;

  // Bail out early if the desired count is smaller than the current count.
  if (segment_landmark_id_set.size() <= desired_num_landmarks) {
    LOG(WARNING) << "Nothing to summarize, bailing out early.";
    summary_landmark_ids->insert(
        segment_landmark_id_set.begin(), segment_landmark_id_set.end());
    return;
  }

  vi_map::LandmarkIdList all_landmark_ids(
      segment_landmark_id_set.begin(), segment_landmark_id_set.end());

  for (unsigned int i = 0; i < all_landmark_ids.size(); ++i) {
    // +1 to keep consistency with lp-solve variable indexing (starts with 1).
    landmark_ids_to_indices_.emplace(all_landmark_ids[i], i + 1);
  }

  // We don't know the exact constraint number right now, the problem will be
  // extended on the fly.
  const unsigned int kInitialConstraintCount = 0;
  // Adding 1 because our state consists of switch variable for each landmark
  // AND a slack variable.
  num_variables_ = all_landmark_ids.size() + 1;

  // Construct lprec object.
  LprecWrapper lprec_ptr;
  lprec_ptr.lprec_ptr = make_lp(kInitialConstraintCount, num_variables_);
  CHECK_NOTNULL(lprec_ptr.lprec_ptr);

  // Set verbose to show only critical messages.
  set_verbose(lprec_ptr.lprec_ptr, CRITICAL);

  std::string lp_name = "ILP landmark summary";
  char* c_lp_name = &lp_name[0u];
  set_lp_name(lprec_ptr.lprec_ptr, c_lp_name);

  addObjectiveFunction(map, lprec_ptr);
  addTotalLandmarksConstraint(desired_num_landmarks, lprec_ptr);

  for (const pose_graph::VertexId& vertex_id : segment_vertex_id_list) {
    vi_map::LandmarkIdList observer_landmarks;
    map.getVertex(vertex_id).getAllObservedLandmarkIds(&observer_landmarks);
    vi_map::LandmarkIdSet landmark_observers;
    for (const vi_map::LandmarkId& landmark_id : observer_landmarks) {
      if (landmark_id.isValid()) {
        landmark_observers.emplace(landmark_id);
      }
    }
    addKeyframeConstraint(
        min_keypoints_per_keyframe_, landmark_observers, lprec_ptr);
  }

  setLandmarkSwitchVariablesToBinary(lprec_ptr);

  // Using Basis Factorization Package (LU decomposition).
  static char kBfpConfigurationString[] = "bfp_LUSOL";
  set_BFP(lprec_ptr.lprec_ptr, kBfpConfigurationString);
  set_scaling(lprec_ptr.lprec_ptr, SCALE_CURTISREID);
  // Example presolver call from lp_solve documentation.
  set_presolve(
      lprec_ptr.lprec_ptr, PRESOLVE_ROWS | PRESOLVE_COLS | PRESOLVE_LINDEP,
      get_presolveloops(lprec_ptr.lprec_ptr));

  // It seems NODE_RANGESELECT does to trick for the optimizer to realize it
  // need to adapt the slack variable. Other flags are default ones for
  // lp_solve.
  set_bb_rule(
      lprec_ptr.lprec_ptr,
      NODE_RANGESELECT | NODE_GREEDYMODE | NODE_DYNAMICMODE | NODE_RCOSTFIXING);

  int ret = solve(lprec_ptr.lprec_ptr);
  switch (ret) {
    case NOMEMORY:
      LOG(ERROR) << "Couldn't solve ILP summarization problem,"
                 << " ran out of memory.";
      return;
      break;
    case OPTIMAL:
      LOG(INFO) << "Optimal solution found.";
      break;
    case SUBOPTIMAL:
      LOG(WARNING) << "Possibly suboptimal solution found.";
      break;
    case INFEASIBLE:
      LOG(ERROR) << "ILP summarization problem infeasible.";
      return;
      break;
    case UNBOUNDED:
      LOG(ERROR) << "ILP summarization problem unbounded.";
      return;
      break;
    case DEGENERATE:
      LOG(ERROR) << "ILP summarization problem degenerate.";
      return;
      break;
    case NUMFAILURE:
      LOG(ERROR) << "Numerical failure when solving ILP summarization problem.";
      return;
      break;
    case TIMEOUT:
      LOG(ERROR) << "ILP summarization solver timeout.";
      return;
      break;
    default:
      LOG(ERROR) << "Solver returned an error with code: " << ret;
      break;
  }

  REAL* variables_ptr;
  get_ptr_variables(lprec_ptr.lprec_ptr, &variables_ptr);
  CHECK_NOTNULL(variables_ptr);

  if (variables_ptr[num_variables_ - 1] > 0) {
    LOG(WARNING) << "Couldn't find a solution for a problem with minimum "
                 << min_keypoints_per_keyframe_
                 << " keypoints per keyframe. Slack variable had to be used, "
                 << "value: " << variables_ptr[num_variables_ - 1];
  }

  unsigned int num_landmarks_left = 0;
  for (const StoreLandmarkIdToIndexMap::value_type& landmark_id_with_index :
       landmark_ids_to_indices_) {
    // Substract 1 as variables_ptr is pointing to lprec struct data array
    // which is indexed from 0 (contrary to row indexing in the problem).
    if (fabs(variables_ptr[landmark_id_with_index.second - 1] - 1.0) <
        std::numeric_limits<float>::epsilon()) {
      summary_landmark_ids->emplace(landmark_id_with_index.first);
      ++num_landmarks_left;
    }
  }
  CHECK_EQ(desired_num_landmarks, num_landmarks_left);

  delete_lp(lprec_ptr.lprec_ptr);
}

void LpSolveSparsification::addKeyframeConstraint(
    unsigned int min_keypoints_per_keyframe,
    const vi_map::LandmarkIdSet& keyframe_landmarks,
    const LprecWrapper& lprec_ptr) const {
  CHECK_NOTNULL(lprec_ptr.lprec_ptr);

  // Add 1 as lp_solve is skipping value at index 0.
  const unsigned int kRowSize = 1 + num_variables_;
  // Initialize the row with kRowSize zeros.
  std::vector<REAL> row(kRowSize, 0.0);

  unsigned int landmarks_in_the_segment = 0;
  for (const vi_map::LandmarkId& landmark_id : keyframe_landmarks) {
    StoreLandmarkIdToIndexMap::const_iterator it =
        landmark_ids_to_indices_.find(landmark_id);
    if (it != landmark_ids_to_indices_.end()) {
      // This landmark is present in the segment being summarized.
      row[it->second] = 1.0;
      ++landmarks_in_the_segment;
    }
  }

  // Slack variable with weight 1.0.
  row[num_variables_] = 1.0;

  // Row mode editing is useful (and recommended) to make the constraint
  // addition faster.
  set_add_rowmode(lprec_ptr.lprec_ptr, TRUE);
  add_constraint(
      lprec_ptr.lprec_ptr, &row.front(), GE,
      std::min(min_keypoints_per_keyframe, landmarks_in_the_segment));
  // Return from row mode to standard mode.
  set_add_rowmode(lprec_ptr.lprec_ptr, FALSE);
}

void LpSolveSparsification::addTotalLandmarksConstraint(
    unsigned int desired_num_landmarks, const LprecWrapper& lprec_ptr) const {
  CHECK_NOTNULL(lprec_ptr.lprec_ptr);

  // Add 1 as lp_solve is skipping value at index 0.
  const unsigned int kRowSize = 1 + num_variables_;
  // Each landmark has cost 0.
  std::vector<REAL> row(kRowSize, 1.0);

  // Slack variable is not contributing to this constraint.
  row[num_variables_] = 0.0;

  set_add_rowmode(lprec_ptr.lprec_ptr, TRUE);
  add_constraint(lprec_ptr.lprec_ptr, &row.front(), LE, desired_num_landmarks);
  set_add_rowmode(lprec_ptr.lprec_ptr, FALSE);
}

void LpSolveSparsification::addObjectiveFunction(
    const vi_map::VIMap& map, const LprecWrapper& lprec_ptr) const {
  CHECK_NOTNULL(lprec_ptr.lprec_ptr);

  const unsigned int kRowSize = 1 + num_variables_;
  std::vector<REAL> row(kRowSize, 0.0);

  const int64_t kSlackVariableCost = 1e10;

  for (const StoreLandmarkIdToIndexMap::value_type& landmark_id_with_index :
       landmark_ids_to_indices_) {
    const vi_map::LandmarkId& landmark_id = landmark_id_with_index.first;

    REAL score = static_cast<REAL>(
        map.getLandmark(landmark_id).getObservations().size());
    row[landmark_id_with_index.second] = score;
  }

  // Slack variable with penalty weight. We maximize our objective so
  // slack variable introduces negative score.
  row[num_variables_] = -kSlackVariableCost;

  set_obj_fn(lprec_ptr.lprec_ptr, &row.front());
  set_maxim(lprec_ptr.lprec_ptr);
}

void LpSolveSparsification::setLandmarkSwitchVariablesToBinary(
    const LprecWrapper& lprec_ptr) const {
  for (unsigned int i = 1; i <= landmark_ids_to_indices_.size(); ++i) {
    set_binary(lprec_ptr.lprec_ptr, i, TRUE);
  }
  // Slack variable is an integer one.
  set_int(lprec_ptr.lprec_ptr, num_variables_, TRUE);
}

}  // namespace map_sparsification
