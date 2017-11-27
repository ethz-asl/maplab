#ifndef MAPLAB_COMMON_ASLAM_ID_PROTO_H_
#define MAPLAB_COMMON_ASLAM_ID_PROTO_H_

#include <glog/logging.h>

#include "maplab-common/id.pb.h"

namespace common {
namespace aslam_id_proto {

static constexpr int kIdNumElements = 2;

// TODO(tcies) duplicate with similar operations in dmap unique id
template <typename IdType>
inline void deserialize(const common::proto::Id& id_field, IdType* id) {
  CHECK_NOTNULL(id);
  CHECK_EQ(2, id_field.uint_size());
  id->fromUint64(id_field.uint().data());
}

template <typename IdType>
inline void serialize(const IdType& id, common::proto::Id* id_field) {
  CHECK_NOTNULL(id_field)->Clear();
  id_field->mutable_uint()->Add();
  id_field->mutable_uint()->Add();
  id.toUint64(id_field->mutable_uint()->mutable_data());
}

}  // namespace aslam_id_proto
}  // namespace common

#endif  // MAPLAB_COMMON_ASLAM_ID_PROTO_H_
