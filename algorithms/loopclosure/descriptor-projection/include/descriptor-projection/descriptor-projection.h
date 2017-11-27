#ifndef DESCRIPTOR_PROJECTION_DESCRIPTOR_PROJECTION_H_
#define DESCRIPTOR_PROJECTION_DESCRIPTOR_PROJECTION_H_

#include <memory>
#include <utility>
#include <vector>

#include <Eigen/Core>
#include <aslam/common/feature-descriptor-ref.h>
#include <aslam/common/memory.h>
#include <aslam/common/unique-id.h>
#include <descriptor-projection/flags.h>
#include <gflags/gflags.h>
#include <glog/logging.h>
#include <loopclosure-common/types.h>
#include <maplab-common/macros.h>
#include <vi-map/unique-id.h>

#include "descriptor-projection/projected_image.pb.h"

namespace loop_closure {
typedef vi_map::MissionId DatasetId;
typedef vi_map::LandmarkId PointLandmarkId;

struct ProjectedImage {
  MAPLAB_POINTER_TYPEDEFS(ProjectedImage);
  int64_t timestamp_nanoseconds;
  KeyframeId keyframe_id;
  DatasetId dataset_id;
  Eigen::MatrixXf projected_descriptors;
  Eigen::Matrix2Xd measurements;
  std::vector<PointLandmarkId> landmarks;

  void serialize(proto::ProjectedImage* projected_image) const;
  void deserialize(const proto::ProjectedImage& projected_image);
};
typedef std::vector<ProjectedImage::Ptr> ProjectedImagePtrList;

}  // namespace loop_closure

namespace descriptor_projection {
typedef Eigen::Matrix<float, Eigen::Dynamic, 1> ProjectedDescriptorType;
typedef Eigen::aligned_allocator<ProjectedDescriptorType> FeatureAllocator;
typedef Aligned<std::vector, ProjectedDescriptorType> DescriptorVector;

typedef std::pair<unsigned int, unsigned int> DescriptorMatch;

typedef Aligned<std::vector, Eigen::Vector3f> CoordinateVector;

template <typename DerivedOut>
void DescriptorToEigenMatrix(
    const aslam::common::FeatureDescriptorConstRef& descriptor,
    const Eigen::MatrixBase<DerivedOut>& matrix_const) {
  EIGEN_STATIC_ASSERT(
      !(Eigen::internal::traits<DerivedOut>::Flags & Eigen::RowMajorBit),
      "This method is only valid for column major matrices");
  Eigen::MatrixBase<DerivedOut>& matrix =
      const_cast<Eigen::MatrixBase<DerivedOut>&>(matrix_const);
  const int num_descriptor_bytes = descriptor.size();
  const int num_descriptor_bits = num_descriptor_bytes * 8;
  CHECK_EQ(matrix.rows(), num_descriptor_bits)
      << "The matrix passed must be preallocated to match the descriptor "
         "length in bits, which is "
      << num_descriptor_bits << ".";

  CHECK_EQ(num_descriptor_bytes % 16, 0);

// Define a set of macros to NEON and SSE instructions so we can use the same
// code further down for both platforms.
#ifdef __ARM_NEON__
#define VECTOR_SET vdupq_n_u8       // Set a vector from a single uint8.
#define VECTOR_LOAD(x) vld1q_u8(x)  // Set a vector from a mem location.
#define VECTOR_TYPE uint8x16_t      // The type of the vector element.
#define VECTOR_AND vandq_u8         // The vector AND instruction.
#define VECTOR_EXTRACT(x, i) vgetq_lane_u8(x, i)  // Get element from vector.
#else
#define VECTOR_SET _mm_set1_epi8
#define VECTOR_LOAD(x) _mm_load_si128(reinterpret_cast<const __m128i*>(x))
#define VECTOR_TYPE __m128i
#define VECTOR_AND _mm_and_si128
// Could use _mm_extract_epi8, but this requires SSE4.1.
#define VECTOR_EXTRACT(x, i) reinterpret_cast<const char*>(&x)[i]
#endif  // __ARM_NEON__

  VECTOR_TYPE mask[8];
  mask[0] = VECTOR_SET((1 << 0));
  mask[1] = VECTOR_SET((1 << 1));
  mask[2] = VECTOR_SET((1 << 2));
  mask[3] = VECTOR_SET((1 << 3));
  mask[4] = VECTOR_SET((1 << 4));
  mask[5] = VECTOR_SET((1 << 5));
  mask[6] = VECTOR_SET((1 << 6));
  mask[7] = VECTOR_SET((1 << 7));

  float* matrix_ref = matrix.derived().data();

  for (int pack = 0; pack < num_descriptor_bytes / 16; ++pack) {
    VECTOR_TYPE value = VECTOR_LOAD(descriptor.data() + pack * 16);
    const int pack128 = pack << 7;
    for (int i = 0; i < 8; ++i) {  // Checks 16 bits at once with SSE/NEON.
      // Masks the i'th bit of the 16 uint8s.
      VECTOR_TYPE xmm1 = VECTOR_AND(value, mask[i]);
      matrix_ref[pack128 + i + 0] = VECTOR_EXTRACT(xmm1, 0) ? 1.f : 0.f;
      matrix_ref[pack128 + i + 8] = VECTOR_EXTRACT(xmm1, 1) ? 1.f : 0.f;
      matrix_ref[pack128 + i + 16] = VECTOR_EXTRACT(xmm1, 2) ? 1.f : 0.f;
      matrix_ref[pack128 + i + 24] = VECTOR_EXTRACT(xmm1, 3) ? 1.f : 0.f;
      matrix_ref[pack128 + i + 32] = VECTOR_EXTRACT(xmm1, 4) ? 1.f : 0.f;
      matrix_ref[pack128 + i + 40] = VECTOR_EXTRACT(xmm1, 5) ? 1.f : 0.f;
      matrix_ref[pack128 + i + 48] = VECTOR_EXTRACT(xmm1, 6) ? 1.f : 0.f;
      matrix_ref[pack128 + i + 56] = VECTOR_EXTRACT(xmm1, 7) ? 1.f : 0.f;
      matrix_ref[pack128 + i + 64] = VECTOR_EXTRACT(xmm1, 8) ? 1.f : 0.f;
      matrix_ref[pack128 + i + 72] = VECTOR_EXTRACT(xmm1, 9) ? 1.f : 0.f;
      matrix_ref[pack128 + i + 80] = VECTOR_EXTRACT(xmm1, 10) ? 1.f : 0.f;
      matrix_ref[pack128 + i + 88] = VECTOR_EXTRACT(xmm1, 11) ? 1.f : 0.f;
      matrix_ref[pack128 + i + 96] = VECTOR_EXTRACT(xmm1, 12) ? 1.f : 0.f;
      matrix_ref[pack128 + i + 104] = VECTOR_EXTRACT(xmm1, 13) ? 1.f : 0.f;
      matrix_ref[pack128 + i + 112] = VECTOR_EXTRACT(xmm1, 14) ? 1.f : 0.f;
      matrix_ref[pack128 + i + 120] = VECTOR_EXTRACT(xmm1, 15) ? 1.f : 0.f;
    }
  }
}

template <typename DerivedIn, typename DerivedOut>
void DescriptorToEigenMatrix(
    const Eigen::MatrixBase<DerivedIn>& descriptor,
    const Eigen::MatrixBase<DerivedOut>& matrix_const) {
  EIGEN_STATIC_ASSERT(
      !(Eigen::internal::traits<DerivedOut>::Flags & Eigen::RowMajorBit),
      "This method is only valid for column major matrices");
  CHECK_EQ(descriptor.cols(), 1);
  Eigen::MatrixBase<DerivedOut>& matrix =
      const_cast<Eigen::MatrixBase<DerivedOut>&>(matrix_const);
  const int num_descriptor_bytes = descriptor.rows();
  const int num_descriptor_bits = num_descriptor_bytes * 8;
  CHECK_EQ(matrix.rows(), num_descriptor_bits)
      << "The matrix passed must be preallocated to match the descriptor "
         "length in bits, which is "
      << num_descriptor_bits << ".";
  matrix.setZero();

  CHECK_EQ(num_descriptor_bytes % 16, 0);

// Define a set of macros to NEON and SSE instructions so we can use the same
// code further down for both platforms.
#ifdef ANDROID
#define VECTOR_SET vdupq_n_u8       // Set a vector from a single uint8.
#define VECTOR_LOAD(x) vld1q_u8(x)  // Set a vector from a mem location.
#define VECTOR_TYPE uint8x16_t      // The type of the vector element.
#define VECTOR_AND vandq_u8         // The vector AND instruction.
#define VECTOR_EXTRACT(x, i) vgetq_lane_u8(x, i)  // Get element from vector.
#else
#define VECTOR_SET _mm_set1_epi8
#define VECTOR_LOAD(x) _mm_load_si128(reinterpret_cast<const __m128i*>(x))
#define VECTOR_TYPE __m128i
#define VECTOR_AND _mm_and_si128
// Could use _mm_extract_epi8, but this requires SSE4.1.
#define VECTOR_EXTRACT(x, i) reinterpret_cast<const char*>(&x)[i]
#endif  // ANDROID

  VECTOR_TYPE mask[8];
  mask[0] = VECTOR_SET((1 << 0));
  mask[1] = VECTOR_SET((1 << 1));
  mask[2] = VECTOR_SET((1 << 2));
  mask[3] = VECTOR_SET((1 << 3));
  mask[4] = VECTOR_SET((1 << 4));
  mask[5] = VECTOR_SET((1 << 5));
  mask[6] = VECTOR_SET((1 << 6));
  mask[7] = VECTOR_SET((1 << 7));

  float* matrix_ref = matrix.derived().data();

  CHECK_EQ(descriptor.derived().cols(), 1);

  const unsigned char* descriptor_data = &descriptor.derived().coeffRef(0, 0);

  for (int pack = 0; pack < num_descriptor_bytes / 16; ++pack) {
    VECTOR_TYPE value = VECTOR_LOAD(descriptor_data + pack * 16);
    const int pack128 = pack << 7;
    for (int i = 0; i < 8; ++i) {  // Checks 16 bits at once with SSE/NEON.
      // Masks the i'th bit of the 16 uint8s.
      VECTOR_TYPE xmm1 = VECTOR_AND(value, mask[i]);
      if (VECTOR_EXTRACT(xmm1, 0))
        matrix_ref[pack128 + i + 0] = 1;
      if (VECTOR_EXTRACT(xmm1, 1))
        matrix_ref[pack128 + i + 8] = 1;
      if (VECTOR_EXTRACT(xmm1, 2))
        matrix_ref[pack128 + i + 16] = 1;
      if (VECTOR_EXTRACT(xmm1, 3))
        matrix_ref[pack128 + i + 24] = 1;
      if (VECTOR_EXTRACT(xmm1, 4))
        matrix_ref[pack128 + i + 32] = 1;
      if (VECTOR_EXTRACT(xmm1, 5))
        matrix_ref[pack128 + i + 40] = 1;
      if (VECTOR_EXTRACT(xmm1, 6))
        matrix_ref[pack128 + i + 48] = 1;
      if (VECTOR_EXTRACT(xmm1, 7))
        matrix_ref[pack128 + i + 56] = 1;
      if (VECTOR_EXTRACT(xmm1, 8))
        matrix_ref[pack128 + i + 64] = 1;
      if (VECTOR_EXTRACT(xmm1, 9))
        matrix_ref[pack128 + i + 72] = 1;
      if (VECTOR_EXTRACT(xmm1, 10))
        matrix_ref[pack128 + i + 80] = 1;
      if (VECTOR_EXTRACT(xmm1, 11))
        matrix_ref[pack128 + i + 88] = 1;
      if (VECTOR_EXTRACT(xmm1, 12))
        matrix_ref[pack128 + i + 96] = 1;
      if (VECTOR_EXTRACT(xmm1, 13))
        matrix_ref[pack128 + i + 104] = 1;
      if (VECTOR_EXTRACT(xmm1, 14))
        matrix_ref[pack128 + i + 112] = 1;
      if (VECTOR_EXTRACT(xmm1, 15))
        matrix_ref[pack128 + i + 120] = 1;
    }
  }
}

template <typename DerivedIn, typename DerivedOut>
inline typename std::enable_if<
    std::is_floating_point<typename DerivedIn::Scalar>::value, void>::type
ProjectDescriptor(
    const Eigen::MatrixBase<DerivedIn>& descriptor,
    const Eigen::MatrixXf& projection_matrix, int target_dimensions,
    const Eigen::MatrixBase<DerivedOut>& projected_descriptor_const) {
  static_assert(
      std::is_same<typename DerivedIn::Scalar, float>::value,
      "This method is only valid for float matrices.");
  Eigen::MatrixBase<DerivedOut>& projected_descriptor =
      const_cast<Eigen::MatrixBase<DerivedOut>&>(projected_descriptor_const);
  CHECK_EQ(projected_descriptor.rows(), target_dimensions);
  projected_descriptor =
      projection_matrix.block(
          0, 0, target_dimensions, projection_matrix.cols()) *
      descriptor.block(0, 0, projection_matrix.cols(), 1);
}

template <typename DerivedIn, typename DerivedOut>
inline typename std::enable_if<
    !std::is_floating_point<typename DerivedIn::Scalar>::value, void>::type
ProjectDescriptor(
    const Eigen::MatrixBase<DerivedIn>& raw_descriptor,
    const Eigen::MatrixXf& projection_matrix, int target_dimensions,
    const Eigen::MatrixBase<DerivedOut>& projected_descriptor_const) {
  static_assert(
      std::is_same<typename DerivedIn::Scalar, unsigned char>::value,
      "This method is only valid for unsigned char matrices.");
  Eigen::MatrixBase<DerivedOut>& projected_descriptor =
      const_cast<Eigen::MatrixBase<DerivedOut>&>(projected_descriptor_const);
  CHECK_EQ(raw_descriptor.cols(), 1);
  Eigen::Matrix<float, Eigen::Dynamic, 1> descriptor;
  const int descriptor_bits = raw_descriptor.rows() * 8;
  descriptor.resize(descriptor_bits, Eigen::NoChange);
  DescriptorToEigenMatrix(raw_descriptor, descriptor);
  CHECK_EQ(projected_descriptor.rows(), target_dimensions);
  projected_descriptor =
      projection_matrix.block(
          0, 0, target_dimensions, projection_matrix.cols()) *
      descriptor.block(0, 0, projection_matrix.cols(), 1);
}

template <typename DerivedOut>
void ProjectDescriptor(
    const aslam::common::FeatureDescriptorConstRef& raw_descriptor,
    const Eigen::MatrixXf& projection_matrix, int target_dimensions,
    const Eigen::MatrixBase<DerivedOut>& projected_descriptor_const) {
  Eigen::MatrixBase<DerivedOut>& projected_descriptor =
      const_cast<Eigen::MatrixBase<DerivedOut>&>(projected_descriptor_const);
  Eigen::Matrix<float, Eigen::Dynamic, 1> descriptor;
  const int descriptor_bits = raw_descriptor.size() * 8;
  descriptor.resize(descriptor_bits, Eigen::NoChange);
  DescriptorToEigenMatrix(raw_descriptor, descriptor);
  ProjectDescriptor(
      descriptor, projection_matrix, target_dimensions, projected_descriptor);
}

void ProjectDescriptorBlock(
    const Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic>&
        raw_descriptors,
    const Eigen::MatrixXf& projection_matrix, int target_dimensions,
    Eigen::MatrixXf* projected_descriptors);

void ProjectDescriptorBlock(
    const std::vector<aslam::common::FeatureDescriptorConstRef>&
        raw_descriptors,
    const Eigen::MatrixXf& projection_matrix, int target_dimensions,
    Eigen::MatrixXf* projected_descriptors);

template <typename DerivedIn>
inline void ProjectDescriptor(
    const Eigen::MatrixBase<DerivedIn>& descriptor,
    const Eigen::MatrixXf& projection_matrix, int target_dimensions,
    std::vector<float>* projected_descriptor) {
  CHECK_NOTNULL(projected_descriptor);
  projected_descriptor->resize(target_dimensions);
  Eigen::Map<Eigen::Matrix<float, Eigen::Dynamic, 1> > descriptor_map(
      projected_descriptor->data(), target_dimensions, 1);

  descriptor_map =
      projection_matrix.block(0, 0, target_dimensions, descriptor.rows()) *
      descriptor;

  Eigen::MatrixXf proj_tmp =
      projection_matrix.block(
          0, 0, target_dimensions, projection_matrix.cols()) *
      descriptor.block(0, 0, projection_matrix.cols(), 1);
}

bool LoadprojectionMatrix(Eigen::MatrixXf* projection_matrix);
}  // namespace descriptor_projection
#endif  // DESCRIPTOR_PROJECTION_DESCRIPTOR_PROJECTION_H_
