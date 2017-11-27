#include "maplab-common/feature-descriptor-ref.h"

#include <aslam/common/feature-descriptor-ref.h>
#include <maplab-common/binary-serialization.h>

namespace common {

void Serialize(
    const aslam::common::FeatureDescriptorRef& value, std::ostream* out) {
  uint32_t size = value.size();
  ::common::Serialize(size, out);
  ::common::Serialize(value.data(), value.size(), out);
}

void Deserialize(aslam::common::FeatureDescriptorRef* value, std::istream* in) {
  CHECK_NOTNULL(value);
  uint32_t size;
  ::common::Deserialize(&size, in);
  value->Allocate(size);
  ::common::Deserialize(value->data(), size, in);
}

}  // namespace common
