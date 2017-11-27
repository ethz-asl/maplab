#ifndef MATCHING_BASED_LOOPCLOSURE_KD_TREE_INDEX_INTERFACE_H_
#define MATCHING_BASED_LOOPCLOSURE_KD_TREE_INDEX_INTERFACE_H_
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <aslam/common/timer.h>
#include <descriptor-projection/descriptor-projection.h>
#include <maplab-common/binary-serialization.h>
#include <matching-based-loopclosure/helpers.h>
#include <matching-based-loopclosure/index-interface.h>
#include <matching-based-loopclosure/kd-tree-index.h>

DECLARE_int32(lc_target_dimensionality);
namespace loop_closure {
using kd_tree_index::KDTreeIndex;
class KDTreeIndexInterface : public IndexInterface {
 public:
  enum { kTargetDimensionality = 10 };
  typedef KDTreeIndex<kTargetDimensionality> Index;

  explicit KDTreeIndexInterface(const std::string& projection_matrix_filepath) {
    std::ifstream deserializer(projection_matrix_filepath);
    CHECK(deserializer.is_open()) << "Cannot load projection matrix from file: "
                                  << projection_matrix_filepath;
    common::Deserialize(&projection_matrix_, &deserializer);

    index_.reset(new Index());
  }

  virtual int GetNumDescriptorsInIndex() const {
    return index_->GetNumDescriptorsInIndex();
  }

  virtual void Clear() {
    index_->Clear();
  }
  virtual void AddDescriptors(const Eigen::MatrixXf& descriptors) {
    CHECK_EQ(descriptors.rows(), kTargetDimensionality);
    std::lock_guard<std::mutex> lock(index_mutex_);
    CHECK(index_ != nullptr);
    index_->AddDescriptors(descriptors);
  }
  template <typename DerivedQuery, typename DerivedIndices,
            typename DerivedDistances>
  inline void GetNNearestNeighbors(
      const Eigen::MatrixBase<DerivedQuery>& query_feature, int num_neighbors,
      const Eigen::MatrixBase<DerivedIndices>& indices_const,
      const Eigen::MatrixBase<DerivedDistances>& distances_const) const {
    EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(
        DerivedQuery, static_cast<int>(kTargetDimensionality), 1);
    CHECK_EQ(indices_const.cols(), 1);
    CHECK_EQ(distances_const.cols(), 1);

    CHECK_EQ(indices_const.rows(), num_neighbors)
        << "The indices parameter must be pre-allocated to hold all results.";
    CHECK_EQ(distances_const.rows(), num_neighbors)
        << "The distances parameter must be pre-allocated to hold all results.";

    Eigen::MatrixXf query_feature_dyn = query_feature;

    std::lock_guard<std::mutex> lock(index_mutex_);
    CHECK(index_ != nullptr);
    index_->GetNNearestNeighbors(
        query_feature_dyn, num_neighbors, indices_const, distances_const);
  }

  virtual void GetNNearestNeighborsForFeatures(
      const Eigen::MatrixXf& query_features, int num_neighbors,
      Eigen::MatrixXi* indices, Eigen::MatrixXf* distances) const {
    CHECK_NOTNULL(indices);
    CHECK_NOTNULL(distances);
    std::lock_guard<std::mutex> lock(index_mutex_);
    CHECK(index_ != nullptr);
    index_->GetNNearestNeighbors(
        query_features, num_neighbors, indices, distances);
  }

  virtual void ProjectDescriptors(
      const DescriptorContainer& descriptors,
      Eigen::MatrixXf* projected_descriptors) const {
    CHECK_NOTNULL(projected_descriptors);
    projected_descriptors->resize(kTargetDimensionality, descriptors.cols());

    timing::Timer timer_proj("PL 1.1 project");
    descriptor_projection::ProjectDescriptorBlock(
        descriptors, projection_matrix_, kTargetDimensionality,
        projected_descriptors);
    timer_proj.Stop();
  }

  virtual void ProjectDescriptors(
      const std::vector<aslam::common::FeatureDescriptorConstRef>& descriptors,
      Eigen::MatrixXf* projected_descriptors) const {
    CHECK_NOTNULL(projected_descriptors);
    projected_descriptors->resize(kTargetDimensionality, descriptors.size());

    timing::Timer timer_proj("PL 1.1 project");
    descriptor_projection::ProjectDescriptorBlock(
        descriptors, projection_matrix_, kTargetDimensionality,
        projected_descriptors);
    timer_proj.Stop();
  }

 private:
  std::shared_ptr<Index> index_;
  Eigen::MatrixXf projection_matrix_;
  mutable std::mutex index_mutex_;
};
}  // namespace loop_closure
#endif  // MATCHING_BASED_LOOPCLOSURE_KD_TREE_INDEX_INTERFACE_H_
