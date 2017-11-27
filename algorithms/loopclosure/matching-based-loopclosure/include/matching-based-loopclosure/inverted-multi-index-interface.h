#ifndef MATCHING_BASED_LOOPCLOSURE_INVERTED_MULTI_INDEX_INTERFACE_H_
#define MATCHING_BASED_LOOPCLOSURE_INVERTED_MULTI_INDEX_INTERFACE_H_
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <aslam/common/timer.h>
#include <descriptor-projection/descriptor-projection.h>
#include <inverted-multi-index/inverted-multi-index.h>
#include <inverted-multi-index/inverted-multi-product-quantization-index.h>
#include <maplab-common/binary-serialization.h>

#include "matching-based-loopclosure/helpers.h"
#include "matching-based-loopclosure/index-interface.h"
#include "matching-based-loopclosure/matching_based_loop_detector.pb.h"

DECLARE_int32(lc_target_dimensionality);

namespace loop_closure {
class InvertedMultiIndexVocabulary {
 public:
  enum { kSerializationVersion = 100 };
  InvertedMultiIndexVocabulary() {
    target_dimensionality_ = FLAGS_lc_target_dimensionality;
  }
  inline void Save(std::ofstream* out_stream) const {
    CHECK_NOTNULL(out_stream);
    int serialized_version = kSerializationVersion;
    common::Serialize(serialized_version, out_stream);
    common::Serialize(target_dimensionality_, out_stream);
    common::Serialize(projection_matrix_, out_stream);
    common::Serialize(words_first_half_, out_stream);
    common::Serialize(words_second_half_, out_stream);
  }
  inline void Load(std::ifstream* in_stream) {
    CHECK_NOTNULL(in_stream);
    int deserialized_version;
    common::Deserialize(&deserialized_version, in_stream);

    int serialized_target_dimensionality;
    common::Deserialize(&serialized_target_dimensionality, in_stream);
    CHECK_EQ(serialized_target_dimensionality, target_dimensionality_);

    common::Deserialize(&projection_matrix_, in_stream);
    common::Deserialize(&words_first_half_, in_stream);
    common::Deserialize(&words_second_half_, in_stream);
  }
  int target_dimensionality_;
  Eigen::MatrixXf words_first_half_;
  Eigen::MatrixXf words_second_half_;
  Eigen::MatrixXf projection_matrix_;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

class InvertedMultiIndexProductVocabulary
    : public InvertedMultiIndexVocabulary {  // NOLINT
 public:
  enum { kSerializationVersionPQ = 200 };
  inline void Save(std::ofstream* out_stream) const {
    CHECK_NOTNULL(out_stream);
    InvertedMultiIndexVocabulary::Save(out_stream);
    int serialized_version = kSerializationVersionPQ;
    common::Serialize(serialized_version, out_stream);
    common::Serialize(number_of_components, out_stream);
    common::Serialize(number_of_centers, out_stream);
    common::Serialize(number_of_dimensions_per_component, out_stream);
    common::Serialize(quantizer_centers_1, out_stream);
    common::Serialize(quantizer_centers_2, out_stream);
  }
  inline void Load(std::ifstream* in_stream) {
    CHECK_NOTNULL(in_stream);
    InvertedMultiIndexVocabulary::Load(in_stream);
    int deserialized_version;
    common::Deserialize(&deserialized_version, in_stream);

    const std::string kQuantizerDrivePath =
        "https://drive.google.com/#folders/0B4P3uq6O1J3VRVBVdGlPeUVUT1E";

    CHECK_EQ(deserialized_version, kSerializationVersionPQ)
        << "This vocabulary file was saved with a different version."
        << "You can download an updated file from: " << kQuantizerDrivePath;

    common::Deserialize(&number_of_components, in_stream);
    common::Deserialize(&number_of_centers, in_stream);
    common::Deserialize(&number_of_dimensions_per_component, in_stream);
    common::Deserialize(&quantizer_centers_1, in_stream);
    common::Deserialize(&quantizer_centers_2, in_stream);
  }
  int number_of_components;
  int number_of_centers;
  int number_of_dimensions_per_component;
  Eigen::MatrixXf quantizer_centers_1;
  Eigen::MatrixXf quantizer_centers_2;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

using inverted_multi_index::InvertedMultiIndex;
class InvertedMultiIndexInterface : public IndexInterface {
 public:
  friend class matching_based_loopclosure::MatchingBasedLoopDetectorSerializer;
  enum { kSubSpaceDimensionality = 5 };
  typedef InvertedMultiIndex<kSubSpaceDimensionality> Index;

  InvertedMultiIndexInterface(
      const std::string& quantizer_filename,
      int num_closest_words_for_nn_search) {
    std::ifstream in(quantizer_filename, std::ios_base::binary);
    CHECK(in.is_open()) << "Failed to read quantizer file from "
                        << quantizer_filename;

    vocabulary_.Load(&in);

    const Eigen::MatrixXf& words_1 = vocabulary_.words_first_half_;
    const Eigen::MatrixXf& words_2 = vocabulary_.words_second_half_;
    CHECK_GT(words_1.cols(), 0);
    CHECK_GT(words_2.cols(), 0);

    CHECK_EQ(kSubSpaceDimensionality, vocabulary_.target_dimensionality_ / 2);

    index_.reset(new Index(words_1, words_2, num_closest_words_for_nn_search));
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
    CHECK_EQ(descriptors.rows(), 2 * kSubSpaceDimensionality);
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
        DerivedQuery, static_cast<int>(2 * kSubSpaceDimensionality), 1);
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
          query_features.template block<2 * kSubSpaceDimensionality, 1>(0, i),
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

  void serialize(
      matching_based_loopclosure::proto::MatchingBasedLoopDetector::
          InvertedMultiIndexInterface* proto_inverted_multi_index_interface)
      const {
    CHECK_NOTNULL(proto_inverted_multi_index_interface);
    CHECK(index_);
    index_->serialize(
        proto_inverted_multi_index_interface->mutable_inverted_multi_index());
  }

  void deserialize(
      const matching_based_loopclosure::proto::MatchingBasedLoopDetector::
          InvertedMultiIndexInterface& proto_inverted_multi_index_interface) {
    CHECK(index_);
    index_->deserialize(
        proto_inverted_multi_index_interface.inverted_multi_index());
  }

 private:
  std::shared_ptr<Index> index_;
  InvertedMultiIndexVocabulary vocabulary_;
};

using inverted_multi_index::InvertedMultiProductQuantizationIndex;
class InvertedMultiProductQuantizationIndexInterface : public IndexInterface {
 public:
  typedef int DataType;
  enum {
    kSubSpaceDimensionality = 5,
    kNumSubSpaceComponents = 5,
    kNumComponents = 2 * kNumSubSpaceComponents,
    kNumDimPerComp = 1,
    kNumCenters = 16
  };
  typedef InvertedMultiProductQuantizationIndex<DataType, kNumComponents,
                                                kNumDimPerComp, kNumCenters>
      Index;

  InvertedMultiProductQuantizationIndexInterface(
      const std::string& quantizer_filename,
      int num_closest_words_for_nn_search) {
    std::ifstream in(quantizer_filename, std::ios_base::binary);
    CHECK(in.is_open()) << "Failed to read quantizer file from "
                        << quantizer_filename;

    vocabulary_.Load(&in);

    const Eigen::MatrixXf& words_1 = vocabulary_.words_first_half_;
    const Eigen::MatrixXf& words_2 = vocabulary_.words_second_half_;

    CHECK_GT(words_1.cols(), 0);
    CHECK_GT(words_2.cols(), 0);

    int number_of_components = vocabulary_.number_of_components;
    int number_of_centers = vocabulary_.number_of_centers;
    int number_of_dimensions_per_component =
        vocabulary_.number_of_dimensions_per_component;
    const Eigen::MatrixXf& quantizer_centers_1 =
        vocabulary_.quantizer_centers_1;
    const Eigen::MatrixXf& quantizer_centers_2 =
        vocabulary_.quantizer_centers_2;

    CHECK_EQ(number_of_components, kNumSubSpaceComponents);
    CHECK_EQ(number_of_centers, kNumCenters);
    CHECK_EQ(number_of_dimensions_per_component, kNumDimPerComp);

    CHECK_EQ(kSubSpaceDimensionality, vocabulary_.target_dimensionality_ / 2);

    index_.reset(
        new Index(
            words_1, words_2, quantizer_centers_1, quantizer_centers_2,
            num_closest_words_for_nn_search));
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
    CHECK_EQ(descriptors.rows(), 2 * kSubSpaceDimensionality);
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
        DerivedQuery, static_cast<int>(2 * kSubSpaceDimensionality), 1);
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
          query_features.template block<2 * kSubSpaceDimensionality, 1>(0, i),
          num_neighbors, indices.col(i), distances.col(i));
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
  InvertedMultiIndexProductVocabulary vocabulary_;
};

}  // namespace loop_closure
#endif  // MATCHING_BASED_LOOPCLOSURE_INVERTED_MULTI_INDEX_INTERFACE_H_
