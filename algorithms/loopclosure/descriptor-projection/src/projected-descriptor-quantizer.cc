#include "descriptor-projection/projected-descriptor-quantizer.h"

#include <descriptor-projection/descriptor-projection.h>
#include <maplab-common/binary-serialization.h>

namespace descriptor_projection {
void ProjectedDescriptorQuantizer::Save(std::ofstream* out_stream) const {
  CHECK_NOTNULL(out_stream);
  int serialized_version = kSerializationVersion;
  common::Serialize(serialized_version, out_stream);
  int serialized_target_dimensionality = target_dimensionality_;
  common::Serialize(serialized_target_dimensionality, out_stream);
  vocabulary_.Save(out_stream);
  common::Serialize(projection_matrix_, out_stream);
  // Serialize the version again at the end.
  common::Serialize(serialized_version, out_stream);
}

bool ProjectedDescriptorQuantizer::Load(std::ifstream* in_stream) {
  CHECK_NOTNULL(in_stream);
  int deserialized_version;
  common::Deserialize(&deserialized_version, in_stream);

  int serialized_target_dimensionality;
  common::Deserialize(&serialized_target_dimensionality, in_stream);
  CHECK_EQ(serialized_target_dimensionality, target_dimensionality_);

  bool tree_ok = vocabulary_.Load(in_stream);
  if (!tree_ok) {
    LOG(ERROR) << "Failed to load tree for vector quantizer.";
    return false;
  }
  common::Deserialize(&projection_matrix_, in_stream);
  common::Deserialize(&deserialized_version, in_stream);
  CHECK_EQ(deserialized_version, kSerializationVersion);
  initialized_ = true;
  return true;
}
}  // namespace descriptor_projection
