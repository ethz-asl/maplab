#ifndef VIO_COMMON_VIO_UPDATE_SERIALIZATION_H_
#define VIO_COMMON_VIO_UPDATE_SERIALIZATION_H_

#include <aslam/cameras/ncamera.h>

#include "vio-common/vio-update.h"
#include "vio-common/vio_update.pb.h"

namespace vio {
namespace serialization {

// Takes a VioUpdate and serializes it into the corresponding Protocol Buffer
// structure. The pointer
// to vio::proto::VioUpdate mustn't be null.
void serializeVioUpdate(
    const vio::VioUpdate& update, vio::proto::VioUpdate* proto);

// Takes a Protocol Buffer and deserializes it into a VioUpdate.
// The pointer to vio::VioUpdate mustn't be null. An optional NCamera can be
// provided which will be
// added into the VisualNFrame.
void deserializeVioUpdate(
    const vio::proto::VioUpdate& proto, vio::VioUpdate* update);
void deserializeVioUpdate(
    const vio::proto::VioUpdate& proto, aslam::NCamera::Ptr n_camera,
    vio::VioUpdate* update);

}  // namespace serialization
}  // namespace vio

#endif  // VIO_COMMON_VIO_UPDATE_SERIALIZATION_H_
