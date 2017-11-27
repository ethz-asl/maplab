#ifndef DENSE_RECONSTRUCTION_ASLAM_CV_INTERFACE_H_
#define DENSE_RECONSTRUCTION_ASLAM_CV_INTERFACE_H_

#include <Eigen/Dense>
#include <aslam/cameras/camera.h>
#include <aslam/common/pose-types.h>

#include "dense-reconstruction/stereo-camera-utils.h"

namespace dense_reconstruction {
namespace stereo {

void getStereoPairFromAslamCvCameras(
    const aslam::Camera& first_camera, const aslam::Camera& second_camera,
    const aslam::Transformation& T_C2_C1, const double scale,
    StereoCameraParameters* stereo_camera_params);

}  // namespace stereo
}  // namespace dense_reconstruction

#endif  // DENSE_RECONSTRUCTION_ASLAM_CV_INTERFACE_H_
