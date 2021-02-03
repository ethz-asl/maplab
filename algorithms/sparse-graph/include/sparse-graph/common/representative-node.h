#ifndef SPARSE_GRAPH_COMMON_REPRESENTATIVE_NODE_H_
#define SPARSE_GRAPH_COMMON_REPRESENTATIVE_NODE_H_

#include <vi-map/vi-map.h>

#include <vector>

namespace spg {

class RepresentativeNode {
 public:
  RepresentativeNode() = default;
  explicit RepresentativeNode(
      const aslam::Transformation& pose, const int64_t timestamp_ns,
      const uint64_t submap_id);

  const aslam::Transformation& getPose() const noexcept;
  const pose_graph::VertexIdList& getVertices() const noexcept;
  uint64_t getAssociatedSubmapId() const noexcept;
  int64_t getTimestampNanoseconds() const noexcept;

  bool isEqualTo(const RepresentativeNode& rhs) const noexcept;
  bool isEarlierThan(const RepresentativeNode& rhs) const noexcept;
  bool isLaterThan(const RepresentativeNode& rhs) const noexcept;

 private:
  aslam::Transformation pose_;
  int64_t timestamp_ns_;
  uint64_t submap_id_;
};

bool operator==(const RepresentativeNode& lhs, const RepresentativeNode& rhs);
bool operator!=(const RepresentativeNode& lhs, const RepresentativeNode& rhs);
bool operator<(const RepresentativeNode& lhs, const RepresentativeNode& rhs);
bool operator>(const RepresentativeNode& lhs, const RepresentativeNode& rhs);

using RepresentativeNodeVector = std::vector<RepresentativeNode>;

}  // namespace spg

#endif  // SPARSE_GRAPH_COMMON_REPRESENTATIVE_NODE_H_
