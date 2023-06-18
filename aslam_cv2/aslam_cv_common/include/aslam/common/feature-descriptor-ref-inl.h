#ifndef ASLAM_COMMON_FEATURE_DESCRIPTOR_REF_INL_H_
#define ASLAM_COMMON_FEATURE_DESCRIPTOR_REF_INL_H_
#include <limits>
#include <random>
#include <vector>

namespace aslam {
namespace common {
template <typename PointerType>
inline FeatureDescriptorRefBase<
    PointerType, ReadOnlyAccessors>::FeatureDescriptorRefBase(PointerType* data,
                                                              uint32_t size)
    : data_(data), size_bytes_(size) {}

template <typename PointerType>
inline FeatureDescriptorRefBase<PointerType, ReadOnlyAccessors>&
FeatureDescriptorRefBase<PointerType, ReadOnlyAccessors>::
operator=(const FeatureDescriptorRefBase<PointerType, ReadOnlyAccessors>& lhs) {
  if (this == &lhs) {
    return *this;
  }
  size_bytes_ = lhs.size_bytes_;
  data_ = lhs.data_;
  return *this;
}
template <typename PointerType>
inline FeatureDescriptorRefBase<PointerType, ReadOnlyAccessors>::
    FeatureDescriptorRefBase(
        const FeatureDescriptorRefBase<PointerType, ReadOnlyAccessors>& lhs) {
  size_bytes_ = lhs.size_bytes_;
  data_ = lhs.data_;
}

template <typename PointerType>
inline FeatureDescriptorRefBase<PointerType, ReadOnlyAccessors>::
    FeatureDescriptorRefBase(
        const FeatureDescriptorRefBase<unsigned char, WriteAccessors>& other) {
  data_ = other.data_;
  size_bytes_ = other.size_bytes_;
}

template <typename PointerType>
inline const unsigned char&
FeatureDescriptorRefBase<PointerType, ReadOnlyAccessors>::
operator[](size_t index) const {
  CHECK_LT(index, size_bytes_);
  return data_[index];
}

template <typename PointerType>
inline bool FeatureDescriptorRefBase<PointerType, ReadOnlyAccessors>::
operator==(const FeatureDescriptorRefBase<PointerType, ReadOnlyAccessors>& lhs)
    const {
  bool same = true;
  same &= static_cast<bool>(data_) == static_cast<bool>(lhs.data_);
  same &= size_bytes_ == lhs.size_bytes_;
  if (data_ && lhs.data_ && same) {
    same &= (memcmp(data_, lhs.data_, size_bytes_) == 0);
  }
  return same;
}

inline FeatureDescriptorRefBase<unsigned char,
                                WriteAccessors>::FeatureDescriptorRefBase()
    : FeatureDescriptorRefBase<unsigned char, ReadOnlyAccessors>(nullptr, 0),
      owned_(false) {}

inline FeatureDescriptorRefBase<unsigned char, WriteAccessors>::
    FeatureDescriptorRefBase(unsigned char* data, size_t size, bool owned)
    : FeatureDescriptorRefBase<unsigned char, ReadOnlyAccessors>(data, size),
      owned_(owned) {}

// Heap allocate a descriptor, this should not be used unless explicitly
// required. Set FeatureDescriptor to a block allocated memory position
// instead.
inline FeatureDescriptorRefBase<
    unsigned char, WriteAccessors>::FeatureDescriptorRefBase(uint32_t size)
    : Base(new unsigned char[size], size), owned_(true) {}

inline void FeatureDescriptorRefBase<unsigned char, WriteAccessors>::Release() {
  if (owned_) {
    delete[] data_;
    data_ = nullptr;
    owned_ = false;
  }
}

inline void FeatureDescriptorRefBase<unsigned char, WriteAccessors>::Allocate(
    size_t size_bytes) {
  if (size_bytes == size_bytes_ && owned_) {
    return;
  }
  Release();
  owned_ = false;
  size_bytes_ = size_bytes;
  data_ = new unsigned char[size_bytes];
  owned_ = true;
}

inline FeatureDescriptorRefBase<unsigned char, WriteAccessors>&
FeatureDescriptorRefBase<unsigned char, WriteAccessors>::
operator=(const FeatureDescriptorRefBase<unsigned char, WriteAccessors>& lhs) {
  if (this == &lhs) {
    return *this;
  }
  // If this instance owns data, we copy from the other instance, no matter if
  // the other instance owns or not. If the other instance owns, we will
  // own data too and copy from it.
  if (owned_ || lhs.owned_) {
    Allocate(lhs.size_bytes_);
    CHECK_EQ(lhs.size_bytes_, size_bytes_);
    memcpy(data_, lhs.data_, size_bytes_);
  } else {
    owned_ = lhs.owned_;
    size_bytes_ = lhs.size_bytes_;
    data_ = lhs.data_;
  }
  return *this;
}

inline FeatureDescriptorRefBase<unsigned char, WriteAccessors>::
    FeatureDescriptorRefBase(
        const FeatureDescriptorRefBase<unsigned char, WriteAccessors>& lhs)
    : Base(nullptr, 0) {
  if (lhs.owned_) {
    Allocate(lhs.size_bytes_);
    CHECK_EQ(lhs.size_bytes_, size_bytes_);
    memcpy(data_, lhs.data_, size_bytes_);
  } else {
    owned_ = lhs.owned_;
    size_bytes_ = lhs.size_bytes_;
    data_ = lhs.data_;
  }
}

inline FeatureDescriptorRefBase<const unsigned char, ReadOnlyAccessors>
FeatureDescriptorRefBase<unsigned char, WriteAccessors>::ToConstRef() const {
  return FeatureDescriptorRefBase<const unsigned char, ReadOnlyAccessors>(
      data_, size_bytes_);
}

inline FeatureDescriptorRefBase<unsigned char,
                                WriteAccessors>::~FeatureDescriptorRefBase() {
  Release();
}

inline void FeatureDescriptorRefBase<unsigned char, WriteAccessors>::SetZero() {
  if (data_ == nullptr) {
    return;
  }
  memset(data_, 0, size_bytes_);
}

inline void FeatureDescriptorRefBase<unsigned char, WriteAccessors>::SetRandom(
    int seed) {
  if (data_ == nullptr) {
    return;
  }
  std::mt19937 gen(seed);
  std::uniform_int_distribution<> dis(
      0, std::numeric_limits<unsigned char>::max());

  for (size_t i = 0; i < size_bytes_; ++i) {
    data_[i] = dis(gen);
  }
}

inline unsigned char*
FeatureDescriptorRefBase<unsigned char, WriteAccessors>::data() {
  return data_;
}

inline void FeatureDescriptorRefBase<unsigned char, WriteAccessors>::set_data(
    unsigned char* data, bool owned) {
  data_ = data;
  owned_ = owned;
}

inline unsigned char& FeatureDescriptorRefBase<unsigned char, WriteAccessors>::
operator[](size_t index) {
  CHECK_LT(index, size_bytes_);
  return data_[index];
}
inline const unsigned char&
FeatureDescriptorRefBase<unsigned char, WriteAccessors>::
operator[](size_t index) const {
  CHECK_LT(index, size_bytes_);
  return data_[index];
}

template <typename TYPE, int ACCESSOR>
void DescriptorMean(
    const std::vector<FeatureDescriptorRefBase<TYPE, ACCESSOR>*>& features,
    FeatureDescriptorRef* mean) {
  if (features.empty()) {
    return;
  }
  CHECK_NOTNULL(mean);
  CHECK_EQ(mean->size(), features[0]->size());
  mean->SetZero();
  std::vector<int> sums;
  sums.resize(features[0]->size() * 8, 0);
  for (const FeatureDescriptorRef* feature : features) {
    for (size_t bit = 0; bit < feature->size() * 8; ++bit) {
      if (GetBit(bit, feature->ToConstRef())) {
        ++sums[bit];
      }
    }
  }
  int half = features.size() / 2;
  for (size_t bit = 0; bit < sums.size(); ++bit) {
    if (sums[bit] > half) {
      SetBit(bit, mean);
    }
  }
}

}  // namespace common
}  // namespace aslam

#endif  // ASLAM_COMMON_FEATURE_DESCRIPTOR_REF_INL_H_
