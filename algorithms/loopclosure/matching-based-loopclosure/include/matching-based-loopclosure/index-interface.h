#ifndef MATCHING_BASED_LOOPCLOSURE_INDEX_INTERFACE_H_
#define MATCHING_BASED_LOOPCLOSURE_INDEX_INTERFACE_H_
#include <vector>

#include <Eigen/Core>
#include <matching-based-loopclosure/helpers.h>

namespace loop_closure {
class IndexInterface {
 public:
  virtual ~IndexInterface() {}

  // Clear the underlying database from all descriptors.
  virtual void Clear() = 0;

  // Use the projection matrix specific to the used index to project the
  // binary descriptors to a lower dimensional, real valued space.
  virtual void ProjectDescriptors(
      const DescriptorContainer& descriptors,
      Eigen::MatrixXf* projected_descriptors) const = 0;

  // The number of individual descriptors in the index.
  virtual int GetNumDescriptorsInIndex() const = 0;

  // Use the projection matrix specific to the used index to project the
  // binary descriptors to a lower dimensional, real valued space.
  virtual void ProjectDescriptors(
      const std::vector<aslam::common::FeatureDescriptorConstRef>& descriptors,
      Eigen::MatrixXf* projected_descriptors) const = 0;

  // Add descriptors to the index. Can be done lazily.
  virtual void AddDescriptors(const Eigen::MatrixXf& descriptors) = 0;

  // Return the indices and distances of the num_neighbors closest descriptors
  // for every descriptor from the query_features matrix.
  virtual void GetNNearestNeighborsForFeatures(
      const Eigen::MatrixXf& query_features, int num_neighbors,
      Eigen::MatrixXi* indices, Eigen::MatrixXf* distances) const = 0;
};
}  // namespace loop_closure
#endif  // MATCHING_BASED_LOOPCLOSURE_INDEX_INTERFACE_H_
