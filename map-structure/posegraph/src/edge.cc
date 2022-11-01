#include <glog/logging.h>
#include <posegraph/edge.h>

namespace pose_graph {

Edge::Edge() : edge_type_(Edge::EdgeType::kUndefined) {}

Edge::Edge(EdgeType edge_type) : edge_type_(edge_type) {}

bool Edge::operator==(const Edge& other) const {
  bool is_same = true;
  is_same &= edge_type_ == other.edge_type_;
  return is_same;
}

void Edge::incidentVertices(std::pair<VertexId, VertexId>* vertices) const {
  CHECK_NOTNULL(vertices);
  vertices->first = from();
  vertices->second = to();
}

std::string Edge::edgeTypeToString(const EdgeType edge_type) {
  switch (edge_type) {
    case EdgeType::kOdometry:
      return "odometry";
      break;
    case EdgeType::kViwls:
      return "viwls";
      break;
    case EdgeType::kLoopClosure:
      return "loop_closure";
      break;
    case EdgeType::kCklamImuLandmark:
      return "cklam_imu_landmark";
      break;
    case EdgeType::kWheelOdometry:
      return "wheel_odometry";
      break;
    default:
      LOG(FATAL) << "No string translation for edge type "
                 << static_cast<int>(edge_type);
      return "";
      break;
  }
  return "";
}

Edge::EdgeType Edge::stringToEdgeType(const std::string& edge_type) {
  if (edge_type.empty()) {
    LOG(FATAL) << "Can't convert edge type string to EdgeType.";
    return EdgeType::kUndefined;
  } else if (edge_type == "loop_closure") {
    return EdgeType::kLoopClosure;
  } else if (edge_type == "odometry") {
    return EdgeType::kOdometry;
  } else if (edge_type == "viwls") {
    return EdgeType::kViwls;
  } else if (edge_type == "cklam_imu_landmark") {
    return EdgeType::kCklamImuLandmark;
  } else if (edge_type == "wheel_odometry") {
    return EdgeType::kWheelOdometry;
  } else {
    LOG(FATAL) << "Unknown edge type string: " << edge_type;
    return EdgeType::kUndefined;
  }
}

}  // namespace pose_graph
