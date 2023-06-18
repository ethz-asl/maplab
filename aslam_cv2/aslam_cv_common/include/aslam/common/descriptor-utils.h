#ifndef VI_MAP_DESCRIPTOR_UTILS_H_
#define VI_MAP_DESCRIPTOR_UTILS_H_

#include <vector>

#include <Eigen/Core>
#include <glog/logging.h>

#include "aslam/common/feature-descriptor-ref.h"
#include "aslam/common/hamming.h"

namespace aslam {
namespace common {
namespace descriptor_utils {

constexpr size_t kBitsPerByte = 8u;

typedef Eigen::Matrix<unsigned char, Eigen::Dynamic, 1> DescriptorType;
typedef Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic>
    DescriptorsType;

inline bool getBit(unsigned int bit, const DescriptorType& descriptor) {
  EIGEN_STATIC_ASSERT_VECTOR_ONLY(DescriptorType);
  int byte = bit / kBitsPerByte;
  int bit_in_byte = bit % kBitsPerByte;
  CHECK_LT(byte, descriptor.rows());

  return descriptor(byte) & (1 << bit_in_byte);
}

inline void setBit(unsigned int bit, DescriptorType* descriptor) {
  CHECK_NOTNULL(descriptor);
  EIGEN_STATIC_ASSERT_VECTOR_ONLY(DescriptorType);
  int byte = bit / kBitsPerByte;
  int bit_in_byte = bit % kBitsPerByte;
  CHECK_LT(byte, descriptor->rows());

  (*descriptor)(byte) |= (1 << bit_in_byte);
}

inline void descriptorMeanRoundedToBinaryValue(
    const DescriptorsType& descriptors, DescriptorType* median) {
  CHECK_GT(descriptors.rows(), 0);
  CHECK_NOTNULL(median)->resize(descriptors.rows(), Eigen::NoChange);
  median->setZero();

  std::vector<int> sums;
  sums.resize(descriptors.rows() * kBitsPerByte, 0);
  const size_t num_descriptor_bits = descriptors.rows() * kBitsPerByte;
  for (int i = 0; i < descriptors.cols(); ++i) {
    for (size_t bit = 0u; bit < num_descriptor_bits; ++bit) {
      if (getBit(bit, descriptors.col(i))) {
        ++sums[bit];
      }
    }
  }
  const int half = descriptors.cols() / 2;
  for (size_t bit = 0u; bit < sums.size(); ++bit) {
    if (sums[bit] > half) {
      setBit(bit, median);
    }
  }
}

// Returns the col-index into raw_descriptors of the descriptor with
// smallest accumulated descriptor distance wrt. all other descriptors in
// raw_descriptors.
inline void getIndexOfDescriptorClosestToMedian(
    const DescriptorsType& raw_descriptors, size_t* median_descriptor_index) {
  const int descriptor_size_bytes = raw_descriptors.rows();
  CHECK_GT(descriptor_size_bytes, 0);
  CHECK_NOTNULL(median_descriptor_index);

  std::vector<aslam::common::FeatureDescriptorConstRef> wrapped_descriptors;
  const int num_descriptors = raw_descriptors.cols();
  wrapped_descriptors.reserve(num_descriptors);

  for (int descriptor_idx = 0; descriptor_idx < num_descriptors;
       ++descriptor_idx) {
    wrapped_descriptors.emplace_back(
        &raw_descriptors.coeffRef(0, descriptor_idx),
        descriptor_size_bytes);
  }

  Eigen::MatrixXi hamming_distance_matrix =
      Eigen::MatrixXi::Zero(num_descriptors, num_descriptors);

  for (int descriptor_idx_row = 0; descriptor_idx_row < num_descriptors;
       ++descriptor_idx_row) {
    const aslam::common::FeatureDescriptorConstRef& row_descriptor =
        wrapped_descriptors[descriptor_idx_row];
    for (int descriptor_idx_col = descriptor_idx_row + 1;
        descriptor_idx_col < num_descriptors; ++descriptor_idx_col) {
      const aslam::common::FeatureDescriptorConstRef& col_descriptor =
          wrapped_descriptors[descriptor_idx_col];

      const int hamming_distance =
          aslam::common::GetNumBitsDifferent(row_descriptor, col_descriptor);
      CHECK_GE(hamming_distance, 0);

      hamming_distance_matrix(descriptor_idx_row, descriptor_idx_col) =
          hamming_distance;
      hamming_distance_matrix(descriptor_idx_col, descriptor_idx_row) =
          hamming_distance;
    }
  }

  Eigen::MatrixXi::Index eigen_median_descriptor_index;
  hamming_distance_matrix.colwise().sum().minCoeff(&eigen_median_descriptor_index);
  CHECK_GE(static_cast<int>(eigen_median_descriptor_index), 0);
  *median_descriptor_index = static_cast<size_t>(eigen_median_descriptor_index);
  CHECK_LT(*median_descriptor_index, static_cast<size_t>(num_descriptors));
}

inline double descriptorMeanAbsoluteDeviation(
    const DescriptorsType& descriptors) {
  if (descriptors.cols() < 2) {
    return 0.;
  }

  DescriptorType mean;
  descriptorMeanRoundedToBinaryValue(descriptors, &mean);

  double deviation = 0.;
  Hamming hamming_norm;
  for (int i = 0; i < descriptors.cols(); ++i) {
    deviation += hamming_norm(descriptors.col(i).data(), mean.data(),
                              descriptors.rows());
  }
  deviation /= descriptors.cols();
  return deviation;
}

template <typename MeanType>
void floatDescriptorMean(const DescriptorsType& descriptors,
                         Eigen::Matrix<MeanType, Eigen::Dynamic, 1>* mean) {
  CHECK_GT(descriptors.rows(), 0);
  CHECK_GT(descriptors.cols(), 0);
  CHECK_NOTNULL(mean)
      ->resize(descriptors.rows() * kBitsPerByte, Eigen::NoChange);
  mean->setZero();

  std::vector<int> sums;
  sums.resize(descriptors.rows() * kBitsPerByte, 0);
  for (int i = 0; i < descriptors.cols(); ++i) {
    for (size_t bit = 0u; bit < descriptors.rows() * kBitsPerByte; ++bit) {
      if (getBit(bit, descriptors.col(i))) {
        ++sums[bit];
      }
    }
  }

  for (size_t bit = 0u; bit < sums.size(); ++bit) {
    (*mean)(bit) = static_cast<MeanType>(sums[bit]) / descriptors.cols();
  }
}

template <typename FloatDescriptorType>
double differenceToMeanSquaredNorm(const DescriptorType& descriptor,
                                   const FloatDescriptorType& mean) {
  EIGEN_STATIC_ASSERT_VECTOR_ONLY(FloatDescriptorType);
  CHECK_EQ(mean.size(), static_cast<int>(descriptor.size() * kBitsPerByte));

  typedef typename FloatDescriptorType::Scalar FloatScalarType;
  Eigen::Matrix<FloatScalarType, Eigen::Dynamic, 1> bits_of_descriptor;
  bits_of_descriptor.resize(descriptor.size() * kBitsPerByte, Eigen::NoChange);
  for (size_t bit = 0u; bit < descriptor.size() * kBitsPerByte; ++bit) {
    if (getBit(bit, descriptor)) {
      bits_of_descriptor[bit] = 1;
    } else {
      bits_of_descriptor[bit] = 0;
    }
  }

  return (bits_of_descriptor - mean).squaredNorm();
}

inline double descriptorMeanStandardDeviation(
    const DescriptorsType& descriptors) {
  if (descriptors.cols() < 2) {
    return 0.;
  }

  Eigen::VectorXd mean;
  mean.resize(descriptors.rows() * kBitsPerByte, Eigen::NoChange);
  floatDescriptorMean(descriptors, &mean);

  double deviation_squared = 0;
  for (int i = 0; i < descriptors.cols(); ++i) {
    deviation_squared += differenceToMeanSquaredNorm(descriptors.col(i), mean);
  }
  deviation_squared /= descriptors.cols();
  return sqrt(deviation_squared);
}

// Returned in order: p1p0, p2p0, p3p0, ... pnp0, p2p1, p3p1, ..., just like in
// matlab pdist.
inline void pairwiseEuclidianDistances(const DescriptorsType& descriptors,
                                       DescriptorType* result) {
  const int num_descriptors = descriptors.cols();
  CHECK_NOTNULL(result)
      ->resize(num_descriptors * (num_descriptors - 1) / 2, Eigen::NoChange);
  int result_index = 0;
  for (int i = 0; i < num_descriptors; ++i) {
    for (int j = i + 1; j < num_descriptors; ++j) {
      (*result)(result_index) =
          (descriptors.col(j) - descriptors.col(i)).norm();
      ++result_index;
    }
  }
}

}  // namespace descriptor_utils
}  // namespace common
}  // namespace aslam

#endif  // VI_MAP_DESCRIPTOR_UTILS_H_
