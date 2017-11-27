#ifndef MATCHING_BASED_LOOPCLOSURE_INVERTED_INDEX_INTERFACE_H_
#define MATCHING_BASED_LOOPCLOSURE_INVERTED_INDEX_INTERFACE_H_
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <aslam/common/timer.h>
#include <descriptor-projection/descriptor-projection.h>
#include <descriptor-projection/flags.h>
#include <maplab-common/binary-serialization.h>
#include <matching-based-loopclosure/helpers.h>
#include <matching-based-loopclosure/index-interface.h>
#include <matching-based-loopclosure/inverted-index.h>

DECLARE_int32(lc_target_dimensionality);

namespace loop_closure {

class InvertedIndexVocabulary {
 public:
  enum { kSerializationVersion = 100 };
  InvertedIndexVocabulary() {
    target_dimensionality_ = FLAGS_lc_target_dimensionality;
  }
  inline void Save(std::ofstream* out_stream) const {
    CHECK_NOTNULL(out_stream);
    int serialized_version = kSerializationVersion;
    common::Serialize(serialized_version, out_stream);
    common::Serialize(target_dimensionality_, out_stream);
    common::Serialize(projection_matrix_, out_stream);
    common::Serialize(words_, out_stream);
  }
  inline void Load(std::ifstream* in_stream) {
    CHECK_NOTNULL(in_stream);
    int deserialized_version;
    common::Deserialize(&deserialized_version, in_stream);

    CHECK_EQ(deserialized_version, kSerializationVersion)
        << "This vocabulary file was saved with a different version.";
    int serialized_target_dimensionality;
    common::Deserialize(&serialized_target_dimensionality, in_stream);
    CHECK_EQ(serialized_target_dimensionality, target_dimensionality_);

    common::Deserialize(&projection_matrix_, in_stream);
    common::Deserialize(&words_, in_stream);
  }
  int target_dimensionality_;
  Eigen::MatrixXf words_;
  Eigen::MatrixXf projection_matrix_;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

using loop_closure::inverted_index::InvertedIndex;
class InvertedIndexInterface : public IndexInterface {
 public:
  enum { kTargetDimensionality = 10 };
  typedef InvertedIndex<kTargetDimensionality> Index;

  InvertedIndexInterface(
      const std::string& quantizer_filename,
      int num_closest_words_for_nn_search) {
    std::ifstream in(quantizer_filename, std::ios_base::binary);
    CHECK(in.is_open()) << "Failed to read quantizer file from "
                        << quantizer_filename;

    vocabulary_.Load(&in);

    const Eigen::MatrixXf& words_ = vocabulary_.words_;
    CHECK_GT(words_.cols(), 0);
    CHECK_EQ(kTargetDimensionality, vocabulary_.target_dimensionality_);

    index_.reset(new Index(words_, num_closest_words_for_nn_search));
  }

  virtual int GetNumDescriptorsInIndex() const {
    return index_->GetNumDescriptorsInIndex();
  }

  virtual void Clear() {
    index_->Clear();
  }
  inline void SetNumClosestWordsForNNSearch(
      int num_closest_words_for_nn_search) {
    index_->SetNumClosestWordsForNNSearch(num_closest_words_for_nn_search);
  }
  virtual void AddDescriptors(const Eigen::MatrixXf& descriptors) {
    CHECK_EQ(descriptors.rows(), kTargetDimensionality);
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

    index_->GetNNearestNeighbors(
        query_feature, num_neighbors, indices_const, distances_const);
  }

  template <typename DerivedQuery, typename DerivedIndices,
            typename DerivedDistances>
  inline void GetNNearestNeighborsForFeatures(
      const Eigen::MatrixBase<DerivedQuery>& query_features, int num_neighbors,
      const Eigen::MatrixBase<DerivedIndices>& indices_const,
      const Eigen::MatrixBase<DerivedDistances>& distances_const) const {
    Eigen::MatrixBase<DerivedIndices>& indices =
        internal::CastConstEigenMatrixToNonConst(indices_const);
    Eigen::MatrixBase<DerivedDistances>& distances =
        internal::CastConstEigenMatrixToNonConst(distances_const);

    CHECK_EQ(indices_const.rows(), num_neighbors)
        << "The indices parameter must be pre-allocated to hold all results.";
    CHECK_EQ(distances_const.rows(), num_neighbors)
        << "The distances parameter must be pre-allocated to hold all results.";
    CHECK_EQ(indices_const.cols(), query_features.cols())
        << "The indices parameter must be pre-allocated to hold all results.";
    CHECK_EQ(distances_const.cols(), query_features.cols())
        << "The distances parameter must be pre-allocated to hold all results.";

    for (int i = 0; i < query_features.cols(); ++i) {
      GetNNearestNeighbors(
          query_features.template block<kTargetDimensionality, 1>(0, i),
          num_neighbors, indices.block(0, i, num_neighbors, 1),
          distances.block(0, i, num_neighbors, 1));
    }
  }

  virtual void GetNNearestNeighborsForFeatures(
      const Eigen::MatrixXf& query_features, int num_neighbors,
      Eigen::MatrixXi* indices, Eigen::MatrixXf* distances) const {
    CHECK_NOTNULL(indices);
    CHECK_NOTNULL(distances);
    GetNNearestNeighborsForFeatures(
        query_features, num_neighbors, *indices, *distances);
  }

  virtual void ProjectDescriptors(
      const DescriptorContainer& descriptors,
      Eigen::MatrixXf* projected_descriptors) const {
    CHECK_NOTNULL(projected_descriptors);
    internal::ProjectDescriptors(
        descriptors, vocabulary_.projection_matrix_,
        vocabulary_.target_dimensionality_, projected_descriptors);
  }

  virtual void ProjectDescriptors(
      const std::vector<aslam::common::FeatureDescriptorConstRef>& descriptors,
      Eigen::MatrixXf* projected_descriptors) const {
    CHECK_NOTNULL(projected_descriptors);
    internal::ProjectDescriptors(
        descriptors, vocabulary_.projection_matrix_,
        vocabulary_.target_dimensionality_, projected_descriptors);
  }

 private:
  std::shared_ptr<Index> index_;
  InvertedIndexVocabulary vocabulary_;
};
}  // namespace loop_closure
#endif  // MATCHING_BASED_LOOPCLOSURE_INVERTED_INDEX_INTERFACE_H_
