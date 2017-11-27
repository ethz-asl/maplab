#include "aslam-serialization/camera-serialization.h"

#include <aslam/cameras/camera-factory.h>
#include <aslam/cameras/camera.h>
#include <aslam/cameras/ncamera.h>
#include <glog/logging.h>
#include <maplab-common/aslam-id-proto.h>
#include <maplab-common/eigen-proto.h>

#include "aslam-serialization/camera.pb.h"

namespace aslam {
namespace serialization {

void serializeCamera(const aslam::Camera& camera, aslam::proto::Camera* proto) {
  CHECK_NOTNULL(proto);

  const int camera_type_int =
      static_cast<std::underlying_type<aslam::Camera::Type>::type>(
          camera.getType());
  const int distortion_type_int =
      static_cast<std::underlying_type<aslam::Distortion::Type>::type>(
          camera.getDistortion().getType());

  CHECK(proto->CameraType_IsValid(camera_type_int))
      << "Camera type " << camera_type_int << " not supported by proto.";
  CHECK(proto->DistortionType_IsValid(distortion_type_int))
      << "Distortion type " << distortion_type_int
      << " not supported by proto.";

  proto->set_camera_type(
      static_cast<proto::Camera::CameraType>(camera_type_int));
  proto->set_distortion_type(
      static_cast<proto::Camera::DistortionType>(distortion_type_int));

  ::common::aslam_id_proto::serialize(camera.getId(), proto->mutable_id());
  proto->set_image_width(camera.imageWidth());
  proto->set_image_height(camera.imageHeight());

  ::common::eigen_proto::serialize(
      camera.getParameters(), proto->mutable_intrinsics());

  ::common::eigen_proto::serialize(
      camera.getDistortion().getParameters(),
      proto->mutable_distortion_parameters());
}

void deserializeCamera(
    const aslam::proto::Camera& proto, aslam::Camera::Ptr* camera) {
  CHECK_NOTNULL(camera);

  aslam::CameraId camera_id;
  Eigen::VectorXd intrinsics;
  Eigen::VectorXd distortion_parameters;
  ::common::eigen_proto::deserialize(proto.intrinsics(), &intrinsics);
  ::common::eigen_proto::deserialize(
      proto.distortion_parameters(), &distortion_parameters);

  // Change max values if camera/distortion models added.
  CHECK_LE(proto.camera_type(), 1);
  CHECK_LE(proto.distortion_type(), 3);

  // Create camera from camera_proto.
  ::common::aslam_id_proto::deserialize(proto.id(), &camera_id);
  *camera = aslam::createCamera(
      camera_id, intrinsics, proto.image_width(), proto.image_height(),
      distortion_parameters,
      static_cast<aslam::Camera::Type>(proto.camera_type()),
      static_cast<aslam::Distortion::Type>(proto.distortion_type()));
}

void serializeNCamera(
    const aslam::NCamera& n_camera, aslam::proto::NCamera* proto) {
  CHECK_NOTNULL(proto);

  ::common::aslam_id_proto::serialize(n_camera.getId(), proto->mutable_id());
  proto->set_label(n_camera.getLabel());

  typedef ::common::proto::SemiStaticMatrixd MatrixProto;
  const int num_cameras = n_camera.getNumCameras();
  for (int camera_idx = 0; camera_idx < num_cameras; ++camera_idx) {
    proto::Camera* camera_proto = CHECK_NOTNULL(proto->add_cameras());
    MatrixProto* transform_proto = CHECK_NOTNULL(proto->add_t_c_i_transforms());

    const aslam::Camera& camera = n_camera.getCamera(camera_idx);
    serializeCamera(camera, camera_proto);
    ::common::eigen_proto::serialize(
        n_camera.get_T_C_B(camera_idx), transform_proto->mutable_data());
  }
  CHECK_EQ(proto->cameras_size(), proto->t_c_i_transforms_size());
}

void deserializeNCamera(
    const aslam::proto::NCamera& proto, aslam::NCamera::Ptr* n_camera) {
  CHECK_NOTNULL(n_camera);

  // Data needed to construct aslam::NCamera.
  aslam::TransformationVector T_C_I_vector;
  std::vector<aslam::Camera::Ptr> camera_vector;

  aslam::NCameraId ncamera_id;
  ::common::aslam_id_proto::deserialize(proto.id(), &ncamera_id);

  const int num_cameras = proto.cameras_size();
  CHECK_EQ(proto.cameras_size(), proto.t_c_i_transforms_size());
  typedef ::common::proto::SemiStaticMatrixd MatrixProto;
  for (int camera_idx = 0; camera_idx < num_cameras; ++camera_idx) {
    const proto::Camera& camera_proto = proto.cameras(camera_idx);
    const MatrixProto& transformation_proto =
        proto.t_c_i_transforms(camera_idx);

    // Add to containers that will be passed to NCamera.
    aslam::Transformation T_C_I;
    ::common::eigen_proto::deserialize(transformation_proto.data(), &T_C_I);
    T_C_I_vector.push_back(T_C_I);
    Camera::Ptr camera;
    deserializeCamera(camera_proto, &camera);
    camera_vector.push_back(camera);

    VLOG(4) << "distort param: "
            << camera->getDistortion().getParameters().transpose();
    VLOG(4) << "camera param: " << camera->getParameters().transpose();
    VLOG(4) << "q_C_I: "
            << T_C_I.getRotation().toImplementation().coeffs().transpose();
    VLOG(4) << "p_C_I: " << T_C_I.getPosition().transpose();
  }
  n_camera->reset(
      new aslam::NCamera(
          ncamera_id, T_C_I_vector, camera_vector, proto.label()));
}

}  // namespace serialization
}  // namespace aslam
