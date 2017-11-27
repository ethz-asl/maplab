#ifndef POSE_GRAPH_MANIPULATION_PLUGIN_RESET_WHEEL_ODOMETRY_H_
#define POSE_GRAPH_MANIPULATION_PLUGIN_RESET_WHEEL_ODOMETRY_H_

namespace vi_map {
  class VIMap;
}

namespace pose_graph_manipulation {

int resetVertexPosesToWheelOdometryTrajectory(vi_map::VIMap *map);

}  // namespace pose_graph_manipulation

#endif  // POSE_GRAPH_MANIPULATION_PLUGIN_RESET_WHEEL_ODOMETRY_H_
