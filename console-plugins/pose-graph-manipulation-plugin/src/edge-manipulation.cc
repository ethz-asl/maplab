#include "pose-graph-manipulation-plugin/edge-manipulation.h"

#include <console-common/command-registerer.h>
#include <glog/logging.h>
#include <maplab-common/conversions.h>
#include <maplab-common/progress-bar.h>
#include <vi-map/vi-map.h>

namespace pose_graph_manipulation {

int assignEdgeUncertainties(
    const vi_map::Edge::EdgeType edge_type,
    const double translation_std_dev_meters,
    const double orientation_std_dev_degrees, vi_map::VIMap* map) {
  CHECK_NOTNULL(map);

  const double orientation_std_dev_rad =
      kDegToRad * orientation_std_dev_degrees;
  Eigen::Matrix<double, 6, 1> diagonal;
  diagonal << translation_std_dev_meters, translation_std_dev_meters,
      translation_std_dev_meters, orientation_std_dev_rad,
      orientation_std_dev_rad, orientation_std_dev_rad;
  const aslam::TransformationCovariance T_A_B_covariance =
      diagonal.asDiagonal();

  pose_graph::EdgeIdList edge_ids;
  map->getAllEdgeIds(&edge_ids);

  LOG(INFO) << "Assigning edge uncertainties...";
  common::ProgressBar progress_bar(edge_ids.size());
  for (const pose_graph::EdgeId& edge_id : edge_ids) {
    CHECK(edge_id.isValid());
    if (map->getEdgeType(edge_id) == edge_type) {
      switch (edge_type) {
        case pose_graph::Edge::EdgeType::kLoopClosure: {
          vi_map::LoopClosureEdge* loop_closure_edge =
              map->getEdgePtrAs<vi_map::LoopClosureEdge>(edge_id);
          CHECK_NOTNULL(loop_closure_edge)
              ->set_T_A_B_Covariance(T_A_B_covariance);
        } break;
        case pose_graph::Edge::EdgeType::kOdometry: {
          vi_map::TransformationEdge* transformation_edge =
              map->getEdgePtrAs<vi_map::TransformationEdge>(edge_id);
          CHECK_NOTNULL(transformation_edge)
              ->set_T_A_B_Covariance_p_q(T_A_B_covariance);
        } break;
        default:
          LOG(FATAL) << "Unsupported edge type: "
                     << static_cast<int>(edge_type);
          break;
      }
    }
    progress_bar.increment();
  }
  LOG(INFO) << "Done";
  return common::kSuccess;
}

int assignSwitchVariableUncertaintiesForLoopClosureEdges(
    const double switch_variable_variance, vi_map::VIMap* map) {
  CHECK_NOTNULL(map);
  CHECK_GT(switch_variable_variance, 0.0);

  pose_graph::EdgeIdList edge_ids;
  map->getAllEdgeIds(&edge_ids);

  LOG(INFO) << "Assigning switch variable uncertainties for loop-closure"
            << " edges...";
  common::ProgressBar progress_bar(edge_ids.size());
  for (const pose_graph::EdgeId& edge_id : edge_ids) {
    CHECK(edge_id.isValid());
    if (map->getEdgeType(edge_id) == vi_map::Edge::EdgeType::kLoopClosure) {
      vi_map::LoopClosureEdge* loop_closure_edge =
          map->getEdgePtrAs<vi_map::LoopClosureEdge>(edge_id);
      CHECK_NOTNULL(loop_closure_edge)
          ->setSwitchVariableVariance(switch_variable_variance);
    }
    progress_bar.increment();
  }
  LOG(INFO) << "Done";
  return common::kSuccess;
}

int assignSwitchVariableValuesForLoopClosureEdges(
    const double switch_variable_value, vi_map::VIMap* map) {
  CHECK_NOTNULL(map);
  CHECK_GE(switch_variable_value, 0.0);
  CHECK_LE(switch_variable_value, 1.0);

  pose_graph::EdgeIdList edge_ids;
  map->getAllEdgeIds(&edge_ids);

  LOG(INFO) << "Assigning switch variable value " << switch_variable_value
            << " to all loop-closure edges...";
  common::ProgressBar progress_bar(edge_ids.size());
  for (const pose_graph::EdgeId& edge_id : edge_ids) {
    CHECK(edge_id.isValid());
    if (map->getEdgeType(edge_id) == vi_map::Edge::EdgeType::kLoopClosure) {
      vi_map::LoopClosureEdge* loop_closure_edge =
          map->getEdgePtrAs<vi_map::LoopClosureEdge>(edge_id);
      CHECK_NOTNULL(loop_closure_edge)
          ->setSwitchVariable(switch_variable_value);
    }
    progress_bar.increment();
  }
  LOG(INFO) << "Done";
  return common::kSuccess;
}

}  // namespace pose_graph_manipulation
