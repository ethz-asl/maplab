#ifndef ASLAM_COMMON_FEATURE_DESCRIPTOR_REF_H_
#define ASLAM_COMMON_FEATURE_DESCRIPTOR_REF_H_
#include <algorithm>
#include <cstdint>
#include <glog/logging.h>
#include <stdlib.h>
#include <vector>

#include "aslam/common/hamming.h"

namespace aslam {
namespace common {
enum AccessorLevels {
  // Read-only access via a member function.
  ReadOnlyAccessors,
  // Read/write access via member functions.
  WriteAccessors
};

template <typename PointerType, int AccessorLevel>
struct FeatureDescriptorRefBase;

// This is a wrapper into a binary feature store, const pointer version.
template <typename PointerType>
struct FeatureDescriptorRefBase<PointerType, ReadOnlyAccessors> {
  typedef PointerType value_type;
  friend class BinaryFeatureStore;

  inline FeatureDescriptorRefBase(PointerType* data, uint32_t size_bytes);

  inline explicit FeatureDescriptorRefBase(
      const FeatureDescriptorRefBase<unsigned char, WriteAccessors>& other);

  inline FeatureDescriptorRefBase(
      const FeatureDescriptorRefBase<PointerType, ReadOnlyAccessors>& lhs);

  inline FeatureDescriptorRefBase<PointerType, ReadOnlyAccessors>& operator=(
      const FeatureDescriptorRefBase<PointerType, ReadOnlyAccessors>& lhs);

  inline ~FeatureDescriptorRefBase() {}

  inline void set_data(PointerType* data) { data_ = data; }
  inline void set_size_bytes(uint32_t size_bytes) { size_bytes_ = size_bytes; }
  inline const unsigned char* data() const { return data_; }
  inline uint32_t size() const { return size_bytes_; }

  inline const unsigned char& operator[](size_t index) const;

  inline bool operator==(const FeatureDescriptorRefBase<
      PointerType, ReadOnlyAccessors>& lhs) const;

 protected:
  PointerType* data_;
  uint32_t size_bytes_;
};

// This is a wrapper into a binary feature store, non const version.
template <>
struct FeatureDescriptorRefBase<
    unsigned char,
    WriteAccessors> : FeatureDescriptorRefBase<unsigned char,
                                               ReadOnlyAccessors> {
  typedef unsigned char value_type;
  typedef FeatureDescriptorRefBase<unsigned char, ReadOnlyAccessors> Base;
  friend class BinaryFeatureStore;

  inline FeatureDescriptorRefBase();
  inline FeatureDescriptorRefBase(unsigned char* data, size_t size_bytes, bool owned);

  // Heap allocate a descriptor, this should not be used unless explicitly
  // required. Set FeatureDescriptor to a block allocated memory position
  // instead.
  inline explicit FeatureDescriptorRefBase(uint32_t size_bytes);
  inline FeatureDescriptorRefBase(
      const FeatureDescriptorRefBase<unsigned char, WriteAccessors>& lhs);

  inline void Release();
  inline void Allocate(size_t size);

  inline FeatureDescriptorRefBase<unsigned char, WriteAccessors>& operator=(
      const FeatureDescriptorRefBase<unsigned char, WriteAccessors>& lhs);
  inline FeatureDescriptorRefBase<const unsigned char, ReadOnlyAccessors>
      ToConstRef() const;

  inline ~FeatureDescriptorRefBase();
  inline void SetZero();
  inline void SetRandom(int seed);
  inline unsigned char* data();
  using Base::data;
  inline void set_data(unsigned char* data, bool owned);

  inline unsigned char& operator[](size_t index);
  inline const unsigned char& operator[](size_t index) const;
  using Base::operator==;  // NOLINT

 private:
  bool owned_;
};

typedef FeatureDescriptorRefBase<const unsigned char, ReadOnlyAccessors>
    FeatureDescriptorConstRef;
typedef FeatureDescriptorRefBase<unsigned char, WriteAccessors>
    FeatureDescriptorRef;

inline void SetBit(size_t bit_position, FeatureDescriptorRef* descriptor) {
  size_t byte = bit_position / 8;
  size_t bit_within_byte = bit_position % 8;
  CHECK_NOTNULL(descriptor);
  CHECK_LT(byte, descriptor->size());
  (*descriptor)[byte] |= 1 << bit_within_byte;
}

inline bool GetBit(size_t bit, const FeatureDescriptorConstRef& descriptor) {
  size_t byte = bit / 8;
  size_t bit_within_byte = bit % 8;
  CHECK_LT(byte, descriptor.size());
  return descriptor[byte] & 1 << bit_within_byte;
}

inline void FlipBit(size_t bit_position, FeatureDescriptorRef* descriptor) {
  size_t byte = bit_position / 8;
  size_t bit_within_byte = bit_position % 8;
  CHECK_NOTNULL(descriptor);
  CHECK_LT(byte, descriptor->size());
  (*descriptor)[byte] ^= static_cast<uint8_t>(1 << bit_within_byte);
}

inline void FlipNRandomBits(size_t num_bits_to_flip, FeatureDescriptorRef* descriptor) {
  CHECK_NOTNULL(descriptor);
  const uint32_t descriptor_size_bytes = descriptor->size();
  const uint32_t descriptor_size_bits = descriptor_size_bytes * 8;
  std::vector<int> bits_to_flip;
  bits_to_flip.reserve(descriptor_size_bits);
  int n(0);
  std::generate_n(
      std::back_inserter(bits_to_flip), descriptor_size_bits, [n]()mutable {return n++;});
  CHECK_LT(num_bits_to_flip, bits_to_flip.size()) << "Cannot flip more than everything.";
  std::random_shuffle(bits_to_flip.begin(), bits_to_flip.end());
  for (size_t i = 0; i < num_bits_to_flip; ++i) {
    FlipBit(bits_to_flip[i], descriptor);
  }
}

template <typename PointerType, int AccessorLevel>
inline size_t GetNumBitsDifferent(
    const FeatureDescriptorRefBase<PointerType, AccessorLevel>& descriptor1,
    const FeatureDescriptorRefBase<PointerType, AccessorLevel>& descriptor2) {
  const uint32_t descriptor_size = descriptor1.size();
  const uint32_t descriptor2_size = descriptor2.size();
  CHECK_EQ(descriptor_size, descriptor2_size) << "Cannot compare descriptors of unequal size.";
  Hamming hamming;
  return static_cast<size_t>(hamming(descriptor1.data(), descriptor2.data(), descriptor_size));
}

template <typename TYPE, int ACCESSOR>
inline void DescriptorMean(
    const std::vector<FeatureDescriptorRefBase<TYPE, ACCESSOR>*>& features,
    FeatureDescriptorRef* mean);

}  // namespace common
}  // namespace aslam

#include "./feature-descriptor-ref-inl.h"
#endif  // ASLAM_COMMON_FEATURE_DESCRIPTOR_REF_H_
