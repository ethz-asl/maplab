#ifndef ASLAM_SERIALIZATION_CAMERA_SERIALIZATION_H_
#define ASLAM_SERIALIZATION_CAMERA_SERIALIZATION_H_

#include <aslam/cameras/camera.h>
#include <aslam/cameras/ncamera.h>

#include "aslam-serialization/camera.pb.h"

namespace aslam {
namespace serialization {

void serializeCamera(const aslam::Camera& camera, aslam::proto::Camera* proto);
void deserializeCamera(
    const aslam::proto::Camera& proto, aslam::Camera::Ptr* camera);

void serializeNCamera(
    const aslam::NCamera& n_camera, aslam::proto::NCamera* proto);
void deserializeNCamera(
    const aslam::proto::NCamera& proto, aslam::NCamera::Ptr* n_camera);

}  // namespace serialization
}  // namespace aslam

#endif  // ASLAM_SERIALIZATION_CAMERA_SERIALIZATION_H_
