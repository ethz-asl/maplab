#ifndef MAPLAB_COMMON_FEATURE_DESCRIPTOR_REF_H_
#define MAPLAB_COMMON_FEATURE_DESCRIPTOR_REF_H_

#include <algorithm>
#include <cstdint>
#include <glog/logging.h>
#include <stdlib.h>
#include <vector>

namespace aslam {
namespace common {
template <typename PointerType, int AccessorLevel>
struct FeatureDescriptorRefBase;
}  // namespace common
}  // namespace aslam

namespace common {

void Serialize(
    const aslam::common::FeatureDescriptorRefBase<unsigned char, 1>& value,
    std::ostream* out);

void Deserialize(
    aslam::common::FeatureDescriptorRefBase<unsigned char, 1>* value,
    std::istream* in);

}  // namespace common

#endif  // MAPLAB_COMMON_FEATURE_DESCRIPTOR_REF_H_
