#include <glog/logging.h>
#include <posegraph/vertex.h>

namespace pose_graph {

void Vertex::incidentEdges(std::unordered_set<EdgeId>* edges) const {
  CHECK_NOTNULL(edges);
  std::unordered_set<EdgeId> incoming;
  getIncomingEdges(&incoming);
  std::unordered_set<EdgeId> outgoing;
  getOutgoingEdges(&outgoing);
  edges->insert(outgoing.begin(), outgoing.end());
  edges->insert(incoming.begin(), incoming.end());
}

} /* namespace pose_graph */
