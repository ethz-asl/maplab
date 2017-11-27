#include "dense-reconstruction/aslam-cv-interface.h"

#include <algorithm>
#include <iostream>
#include <iterator>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <aslam/cameras/camera-pinhole.h>
#include <aslam/cameras/camera.h>
#include <aslam/cameras/distortion.h>
#include <aslam/common/pose-types.h>

#include "dense-reconstruction/stereo-camera-utils.h"

namespace dense_reconstruction {
namespace stereo {

void getDistortionFromAslamCvCameras(
    const aslam::Camera& camera, std::vector<double>* D,
    DistortionModel* model) {
  CHECK_NOTNULL(model);
  CHECK_NOTNULL(D);

  const aslam::Distortion& distortion = camera.getDistortion();

  *D = std::vector<double>(5, 0.0);

  switch (distortion.getType()) {
    case aslam::Distortion::Type::kNoDistortion:
      *model = DistortionModel::RADTAN;
      return;
    case aslam::Distortion::Type::kEquidistant:
      *model = DistortionModel::EQUIDISTANT;
      break;
    case aslam::Distortion::Type::kFisheye:
      *model = DistortionModel::FOV;
      break;
    case aslam::Distortion::Type::kRadTan:
      *model = DistortionModel::RADTAN;
      break;
    default:
      LOG(FATAL) << "Unknown distortion model!";
  }
  const Eigen::VectorXd distortion_params = distortion.getParameters();
  CHECK_LE(distortion_params.size(), 5);
  for (int i = 0; i < distortion_params.size(); ++i) {
    (*D)[i] = distortion_params[i];
  }
}

void getStereoPairFromAslamCvCameras(
    const aslam::Camera& first_camera, const aslam::Camera& second_camera,
    const aslam::Transformation& T_C2_C1, const double scale,
    StereoCameraParameters* stereo_camera_params) {
  CHECK_NOTNULL(stereo_camera_params);

  *stereo_camera_params = StereoCameraParameters(scale);

  // We assume frame G == C1 for the stereo matching.
  // Therefore T_C1_C2 == T_G_C2 and T_G_C1 = Identity.
  const Eigen::Matrix4d T_G_C2_mat = T_C2_C1.getTransformationMatrix();
  const Eigen::Matrix4d T_G_C1_mat = Eigen::Matrix4d::Identity();

  Eigen::Matrix3d K_first, K_second;
  if (first_camera.getType() == aslam::Camera::Type::kPinhole) {
    K_first = static_cast<const aslam::PinholeCamera&>(first_camera)
                  .getCameraMatrix();

  } else {
    LOG(FATAL) << "Currently only the pinhole camera model is supported!";
  }
  if (second_camera.getType() == aslam::Camera::Type::kPinhole) {
    K_second = static_cast<const aslam::PinholeCamera&>(second_camera)
                   .getCameraMatrix();

  } else {
    LOG(FATAL) << "Currently only the pinhole camera model is supported!";
  }

  const cv::Size resolution_first(
      first_camera.imageWidth(), first_camera.imageHeight());
  const cv::Size resolution_second(
      second_camera.imageWidth(), second_camera.imageHeight());

  std::vector<double> D_first, D_second;
  DistortionModel distortion_model_first, distortion_model_second;
  getDistortionFromAslamCvCameras(
      first_camera, &D_first, &distortion_model_first);
  getDistortionFromAslamCvCameras(
      second_camera, &D_second, &distortion_model_second);

  CHECK(
      stereo_camera_params->setInputCameraParameters(
          resolution_first, T_G_C1_mat, K_first, D_first,
          distortion_model_first, CameraSide::FIRST));
  CHECK(
      stereo_camera_params->setInputCameraParameters(
          resolution_second, T_G_C2_mat, K_second, D_second,
          distortion_model_second, CameraSide::SECOND));
}

}  // namespace stereo
}  // namespace dense_reconstruction
