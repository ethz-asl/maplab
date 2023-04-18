#ifndef PRODUCT_QUANTIZATION_PRODUCT_QUANTIZATION_H_
#define PRODUCT_QUANTIZATION_PRODUCT_QUANTIZATION_H_

#include <limits>

#include <Eigen/Core>
#include <glog/logging.h>

namespace product_quantization {
using Eigen::Block;
using Eigen::Dynamic;
using Eigen::Matrix;
using Eigen::RowMajor;
using Eigen::VectorBlock;

// Implementation of product quantization: A vector is split into kNumComponents
// components of equal length, each of which is quantized individually by
// assigning the subvectors to their closest cluster centers. The quantized
// vector is then represented by the indices of the respective cluster centers.
// This representation has two advantages: First, it allows us to obtain a more
// compact vector representation. For example, using 8 components of 16
// dimensions each, each of which is quantized using 256 cluster centers, a 128
// dimensional SIFT descriptor can be stored using 8 byte instead of 128 byte.
// Second, it enables us to compute the (approximate) distances between a
// query vector and a set of product quantized descriptors more efficiently:
// After computing and storing the distances between each component of the query
// vector and the corresponding cluster centers, computing the distance between
// the query and a quantized vector requires only table lookups and additions.
// The template parameters are the number of components, the dimension per
// component, the number of cluster centers per component used for quantization,
// and the data type used to store the indices.
template <int kNumComponents, int kNumDimPerComp, int kNumCenters,
          typename IndexType>
class ProductQuantization {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  typedef Matrix<float, kNumComponents * kNumDimPerComp, 1> VectorType;
  typedef Matrix<float, kNumComponents * kNumDimPerComp, Dynamic>
      VectorMatrixType;
  typedef Matrix<float, kNumDimPerComp, kNumComponents * kNumCenters>
      ClusterType;
  typedef Matrix<IndexType, kNumComponents, 1> QuantizedVectorType;
  typedef Matrix<IndexType, kNumComponents, Dynamic> QuantizedVectorMatrixType;

  ProductQuantization() {
    static_assert(
        std::numeric_limits<IndexType>::max() >= kNumCenters,
        "The IndexType is not large enough.");
  }

  // The first kNumCenters columns define the cluster centers of the first
  // component, the next kNumCenters define the centers for the next component,
  // and so on.
  explicit ProductQuantization(const ClusterType& cluster_centers)
      : cluster_centers_(cluster_centers) {
    static_assert(
        std::numeric_limits<IndexType>::max() >= kNumCenters,
        "The IndexType is not large enough.");
  }

  void SetClusterCenters(const ClusterType& cluster_centers) {
    cluster_centers_ = cluster_centers;
  }

  // Quantizes a set of vectors, given by the columns of the input matrix. On
  // return, the columns of quantized_vectors give the quantized vectors.
  void Quantize(
      const VectorMatrixType& vectors,
      QuantizedVectorMatrixType* quantized_vectors) const {
    CHECK_NOTNULL(quantized_vectors);
    CHECK_GT(vectors.cols(), 0);
    quantized_vectors->resize(Eigen::NoChange, vectors.cols());

    QuantizedVectorType quantized_vec;
    for (int i = 0; i < vectors.cols(); ++i) {
      Quantize(vectors.col(i), &quantized_vec);
      quantized_vectors->col(i) = quantized_vec;
    }
  }

  void Quantize(
      const VectorType& vector, QuantizedVectorType* quantized_vector) const {
    CHECK_NOTNULL(quantized_vector);
    QuantizedVectorType& quantized_vector_ref = *quantized_vector;

    int center_index = 0;
    int component_index = 0;
    for (int j = 0; j < kNumComponents;
         ++j, component_index += kNumDimPerComp, center_index += kNumCenters) {
      Eigen::MatrixXf::Index nn_index;
      const Matrix<float, kNumDimPerComp, 1>& component =
          vector.template segment<kNumDimPerComp>(component_index);

      const Matrix<float, kNumDimPerComp, kNumCenters>& centers =
          cluster_centers_.template block<kNumDimPerComp, kNumCenters>(
              0, center_index);
      (centers.colwise() - component)
          .colwise()
          .squaredNorm()
          .minCoeff(&nn_index);
      quantized_vector_ref[j] = static_cast<IndexType>(nn_index);
    }
  }

  // Fills a look-up table that contains the squared Euclidean distance between
  // each component of the input vector and its corresponding cluster centers.
  void FillLUT(
      const VectorType& vector,
      Matrix<float, kNumComponents, kNumCenters>* lut) const {
    CHECK_NOTNULL(lut);
    lut->resize(kNumComponents, kNumCenters);

    int center_index = 0;
    int component_index = 0;
    for (int i = 0; i < kNumComponents;
         ++i, component_index += kNumDimPerComp, center_index += kNumCenters) {
      const Matrix<float, kNumDimPerComp, 1>& component =
          vector.template segment<kNumDimPerComp>(component_index);

      const Matrix<float, kNumDimPerComp, kNumCenters>& centers =
          cluster_centers_.template block<kNumDimPerComp, kNumCenters>(
              0, center_index);
      (*lut).row(i) = (centers.colwise() - component).colwise().squaredNorm();
    }
  }

  // Computes and returns the squared Euclidean distances between the query
  // vector and a set of quantized vectors using the look-up table computes by
  // FillLUT. The i-th entry of the squared_distances vector gives the distance
  // to the i-th quantized vector.
  void ComputeDistances(
      const Matrix<float, kNumComponents, kNumCenters>& lut,
      const QuantizedVectorMatrixType& quantized_vectors,
      Matrix<float, 1, Dynamic>* squared_distances) const {
    CHECK_NOTNULL(squared_distances);
    Matrix<float, 1, Dynamic>& squared_distances_ref = *squared_distances;
    squared_distances_ref.resize(Eigen::NoChange, quantized_vectors.cols());

    for (int i = 0; i < quantized_vectors.cols(); ++i) {
      squared_distances_ref[i] = ComputeDistance(lut, quantized_vectors.col(i));
    }
  }

  inline float ComputeDistance(
      const Matrix<float, kNumComponents, kNumCenters>& lut,
      const QuantizedVectorType& quantized_vectors) const {
    float dist = 0.0f;
    for (int j = 0; j < kNumComponents; ++j) {
      dist += lut(j, quantized_vectors[j]);
    }
    return dist;
  }

  // Computes the squared Euclidean distances between the query vector and a set
  // of quantized vectors using the look-up table computes by FillLUT.
  // The computed distances are then added to the existing distances. This
  // assumes that squared_distances contains as many columns as
  // quantized_vectors.
  void ComputeAndAddDistances(
      const Matrix<float, kNumComponents, kNumCenters>& lut,
      const QuantizedVectorMatrixType& quantized_vectors,
      Matrix<float, 1, Dynamic>* squared_distances) const {
    CHECK_NOTNULL(squared_distances);
    CHECK_EQ(squared_distances->cols(), quantized_vectors.cols());
    Matrix<float, 1, Dynamic>& squared_distances_ref = *squared_distances;

    for (int i = 0; i < quantized_vectors.cols(); ++i) {
      for (int j = 0; j < kNumComponents; ++j) {
        squared_distances_ref[i] += lut(j, quantized_vectors(j, i));
      }
    }
  }

 private:
  ClusterType cluster_centers_;
};

}  // namespace product_quantization

#endif  // PRODUCT_QUANTIZATION_PRODUCT_QUANTIZATION_H_
