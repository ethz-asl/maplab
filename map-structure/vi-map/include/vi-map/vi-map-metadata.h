#ifndef VI_MAP_VI_MAP_METADATA_H_
#define VI_MAP_VI_MAP_METADATA_H_

#include <map>
#include <string>
#include <utility>

#include <glog/logging.h>
#include <maplab-common/macros.h>

#include "vi-map/vi_map.pb.h"

namespace vi_map {
namespace serialization {

// Changing the order of this will break existing VI maps as the mapping from
// proto int to enum will be different!
enum class VIMapFileType : int32_t {
  kMissions = 0,
  kVertices = 1,
  kEdges = 2,
  kLandmarkIndex = 3,
  kOptionalSensorData = 10
};

typedef std::multimap<VIMapFileType, std::string> VIMapMetadata;
typedef std::pair<VIMapMetadata::iterator, VIMapMetadata::iterator>
    VIMapMetadataRange;
typedef std::pair<VIMapMetadata::const_iterator, VIMapMetadata::const_iterator>
    VIMapMetadataConstRange;

inline void deserializeMetadata(
    const proto::VIMapMetadata& proto, VIMapMetadata* metadata) {
  CHECK_NOTNULL(metadata)->clear();
  const int num_files = proto.files_size();
  for (int i = 0; i < num_files; ++i) {
    const proto::FileTypeWithPath& file_type_with_path = proto.files(i);
    const std::string& path_from_proto = file_type_with_path.path();
    CHECK(!path_from_proto.empty());
    metadata->emplace(
        static_cast<VIMapFileType>(file_type_with_path.file_type()),
        path_from_proto);
  }
}

inline void serializeMetadata(
    const VIMapMetadata& metadata, proto::VIMapMetadata* proto) {
  CHECK_NOTNULL(proto)->Clear();
  for (const VIMapMetadata::value_type& file_type_with_path : metadata) {
    proto::FileTypeWithPath* file_type_with_path_proto = proto->add_files();
    CHECK_NOTNULL(file_type_with_path_proto);
    file_type_with_path_proto->set_file_type(
        static_cast<int32_t>(file_type_with_path.first));
    const std::string& path = file_type_with_path.second;
    CHECK(!path.empty());
    file_type_with_path_proto->set_path(path);
  }
}

}  // namespace serialization
}  // namespace vi_map

#endif  // VI_MAP_VI_MAP_METADATA_H_
