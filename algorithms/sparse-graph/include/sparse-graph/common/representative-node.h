#ifndef SPARSE_GRAPH_COMMON_REPRESENTATIVE_NODE_H_
#define SPARSE_GRAPH_COMMON_REPRESENTATIVE_NODE_H_

#include <vi-map/vi-map.h>

#include <vector>

#include <resources-common/point-cloud.h>

namespace spg {

class RepresentativeNode {
 public:
  RepresentativeNode() = default;
  explicit RepresentativeNode(
      const aslam::Transformation& pose, const int64_t timestamp_ns,
      const uint32_t submap_id);

  const aslam::Transformation& getPose() const noexcept;
  uint32_t getAssociatedSubmapId() const noexcept;
  int64_t getTimestampNanoseconds() const noexcept;

  double& getResidual();
  double getResidual() const noexcept;
  void setResidual(const double res);

  bool isEqualTo(const RepresentativeNode& rhs) const noexcept;
  bool isEarlierThan(const RepresentativeNode& rhs) const noexcept;
  bool isLaterThan(const RepresentativeNode& rhs) const noexcept;
  bool isActive() const noexcept;

  double distanceTo(const RepresentativeNode& rhs) const noexcept;
  aslam::Transformation transformTo(const RepresentativeNode& rhs) const
      noexcept;

  std::vector<uint32_t> getLocalIndex() const noexcept;
  void setLocalIndex(const std::vector<uint32_t>& local_index);
  void setLocalIndex(std::vector<uint32_t>&& local_index);
  bool containsLocalIndex(const uint32_t local_index) const noexcept;

  bool hasPointCloud() const noexcept;
  void setPointCloud(const resources::PointCloud& pc);
  resources::PointCloud getPointCloud() const noexcept;

 private:
  aslam::Transformation pose_;
  int64_t timestamp_ns_;
  uint32_t submap_id_;
  double residual_;
  bool is_active_ = false;
  std::vector<uint32_t> local_index_;
  resources::PointCloud point_cloud_;
};

bool operator==(const RepresentativeNode& lhs, const RepresentativeNode& rhs);
bool operator!=(const RepresentativeNode& lhs, const RepresentativeNode& rhs);
bool operator<(const RepresentativeNode& lhs, const RepresentativeNode& rhs);
bool operator>(const RepresentativeNode& lhs, const RepresentativeNode& rhs);

using RepresentativeNodeVector = std::vector<RepresentativeNode>;

}  // namespace spg

#endif  // SPARSE_GRAPH_COMMON_REPRESENTATIVE_NODE_H_
