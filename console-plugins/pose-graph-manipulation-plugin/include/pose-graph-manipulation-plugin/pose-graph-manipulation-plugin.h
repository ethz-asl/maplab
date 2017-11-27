#ifndef POSE_GRAPH_MANIPULATION_PLUGIN_POSE_GRAPH_MANIPULATION_PLUGIN_H_
#define POSE_GRAPH_MANIPULATION_PLUGIN_POSE_GRAPH_MANIPULATION_PLUGIN_H_

#include <string>

#include <console-common/console.h>

namespace pose_graph_manipulation {
class PoseGraphManipulationPlugin : public common::ConsolePluginBase {
 public:
  explicit PoseGraphManipulationPlugin(common::Console* console);

  std::string getPluginId() const override {
    return "posegraphmanipulation";
  }

 private:
  // Resets the vertex poses by integrating the relative pose
  // information stored in the odometry edges of the pose graph.
  int resetVertexPosesToWheelOdometryTrajectory() const;
  int assignEdgeUncertainties();
  int assignSwitchVariableUncertaintiesForLoopClosureEdges();
  int assignSwitchVariableValuesForLoopClosureEdges();
};
}  // namespace pose_graph_manipulation

#endif  // POSE_GRAPH_MANIPULATION_PLUGIN_POSE_GRAPH_MANIPULATION_PLUGIN_H_
