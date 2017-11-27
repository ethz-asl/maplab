#ifndef POSE_GRAPH_MANIPULATION_PLUGIN_EDGE_MANIPULATION_H_
#define POSE_GRAPH_MANIPULATION_PLUGIN_EDGE_MANIPULATION_H_

#include <vi-map/edge.h>

namespace vi_map {
class VIMap;
}

namespace pose_graph_manipulation {

int assignEdgeUncertainties(
    const vi_map::Edge::EdgeType edge_type,
    const double translation_std_dev_meters,
    const double orientation_std_dev_degrees, vi_map::VIMap* map);

int assignSwitchVariableUncertaintiesForLoopClosureEdges(
    const double switch_variable_std_dev, vi_map::VIMap* map);
int assignSwitchVariableValuesForLoopClosureEdges(
    const double switch_variable_value, vi_map::VIMap* map);

}  // namespace pose_graph_manipulation

#endif  // POSE_GRAPH_MANIPULATION_PLUGIN_EDGE_MANIPULATION_H_
