#include "rovioli/rovio-factory.h"

#include <Eigen/Core>
#include <aslam/cameras/camera-pinhole.h>
#include <aslam/cameras/camera.h>
#include <aslam/cameras/distortion.h>
#include <gflags/gflags.h>
#include <glog/logging.h>
#include <maplab-common/file-system-tools.h>
#include <message-flow/message-flow.h>
#include <rovio/FilterConfiguration.hpp>
#include <rovio/RovioInterfaceBuilder.hpp>
#include <vio-common/vio-types.h>

DEFINE_bool(
    rovio_enable_frame_visualization, true,
    "Set to false to disable the Rovio GUI.");

namespace rovioli {
namespace {
template <int kNumCameras>
struct RovioBuilder {
  // kLocalizationMode: 0-off, 1: estimate baseframe, 2: (1) + sensor offset
  static constexpr int kLocalizationMode = 1;
  // Maximal number of considered features in the filter state.
  static constexpr int kMaxNumFeatures = 25;
  // Total number of pyramid levels considered.
  static constexpr int kPyramidLevels = 4;
  // Edge length of the patches (in pixel). Must be a multiple of 2!
  static constexpr int kFeaturePatchSizePx = 6;
  rovio::RovioInterface* operator()(
      const rovio::FilterConfiguration& filter_config,
      const rovio::CameraCalibrationVector& camera_calibrations) {
    return rovio::createRovioInterface<kNumCameras, kLocalizationMode,
                                       kMaxNumFeatures, kPyramidLevels,
                                       kFeaturePatchSizePx>(
        filter_config, camera_calibrations);
  }
};

void convertAslamToRovioCamera(
    const aslam::NCamera& aslam_cameras,
    rovio::CameraCalibrationVector* rovio_cameras) {
  CHECK_NOTNULL(rovio_cameras)->clear();

  const size_t num_cameras = aslam_cameras.numCameras();
  for (size_t cam_idx = 0u; cam_idx < num_cameras; ++cam_idx) {
    rovio::CameraCalibration rovio_camera;

    // Translate cameras intrinsic parameters.
    const aslam::Camera& aslam_camera = aslam_cameras.getCamera(cam_idx);
    CHECK_EQ(aslam_camera.getType(), aslam::Camera::Type::kPinhole)
        << "Only pinhole models supported.";
    rovio_camera.K_ = static_cast<const aslam::PinholeCamera&>(aslam_camera)
                          .getCameraMatrix();

    // Translate cameras distortion parameters.
    switch (aslam_camera.getDistortion().getType()) {
      case aslam::Distortion::Type::kNoDistortion:
        // The no-distortion case is emulated using a RADTAN distortion with
        // parameters that do not distort anything.
        rovio_camera.distortionModel_ = rovio::DistortionModel::RADTAN;
        rovio_camera.distortionParams_ = Eigen::Matrix<double, 5, 1>::Zero();
        break;
      case aslam::Distortion::Type::kRadTan:
        rovio_camera.distortionModel_ = rovio::DistortionModel::RADTAN;
        // Rovio uses 5 parameters (k1,k2,p1,p2,k3) we only use 4 (k1,k2,p1,p2)
        // therefore we pad with one zero.
        rovio_camera.distortionParams_.resize(5);
        rovio_camera.distortionParams_.head<4>() =
            aslam_camera.getDistortion().getParameters();
        rovio_camera.distortionParams_(4) = 0.0;
        break;
      case aslam::Distortion::Type::kEquidistant:
        rovio_camera.distortionModel_ = rovio::DistortionModel::EQUIDIST;
        rovio_camera.distortionParams_ =
            aslam_camera.getDistortion().getParameters();
        break;
      case aslam::Distortion::Type::kFisheye:
        rovio_camera.distortionModel_ = rovio::DistortionModel::FOV;
        rovio_camera.distortionParams_ =
            aslam_camera.getDistortion().getParameters();
        break;
      default:
        LOG(FATAL) << "Unsupported distortion.";
        break;
    }
    rovio_camera.hasIntrinsics_ = true;

    // Translate extrinsics.
    const aslam::Transformation T_C_B = aslam_cameras.get_T_C_B(cam_idx);
    const Eigen::Vector3d MrMC = T_C_B.getPosition();
    const kindr::RotationQuaternionPD qCM(
        T_C_B.getRotation().toImplementation());
    rovio_camera.setCameraExtrinsics(MrMC, qCM);
    rovio_cameras->emplace_back(rovio_camera);
  }
  CHECK_EQ(rovio_cameras->size(), num_cameras);
}

void initFilterConfigurationFromGFLags(
    rovio::FilterConfiguration* rovio_config) {
  CHECK_NOTNULL(rovio_config);
  rovio_config->setDoFrameVisualization(FLAGS_rovio_enable_frame_visualization);
}

void setRovioImuSigmas(
    const vi_map::ImuSigmas& maplab_imu_sigmas,
    rovio::FilterConfiguration* rovio_config) {
  CHECK_NOTNULL(rovio_config);
  CHECK(maplab_imu_sigmas.isValid());

  // TODO(schneith): Double check the noise parameters.
  // TODO(schneith): Add Sigma->Cov conversion.

  // ROVIO default: 1e-4
  rovio_config->setAccCovarianceX(maplab_imu_sigmas.acc_noise_density);
  rovio_config->setAccCovarianceY(maplab_imu_sigmas.acc_noise_density);
  rovio_config->setAccCovarianceZ(maplab_imu_sigmas.acc_noise_density);

  // ROVIO default: 1e-8
  rovio_config->setAccBiasCovarianceX(
      maplab_imu_sigmas.acc_bias_random_walk_noise_density);
  rovio_config->setAccBiasCovarianceY(
      maplab_imu_sigmas.acc_bias_random_walk_noise_density);
  rovio_config->setAccBiasCovarianceZ(
      maplab_imu_sigmas.acc_bias_random_walk_noise_density);

  // ROVIO default: 7.6e-7
  rovio_config->setGyroCovarianceX(maplab_imu_sigmas.gyro_noise_density);
  rovio_config->setGyroCovarianceY(maplab_imu_sigmas.gyro_noise_density);
  rovio_config->setGyroCovarianceZ(maplab_imu_sigmas.gyro_noise_density);

  // ROVIO default: 3.8e-7
  rovio_config->setGyroBiasCovarianceX(
      maplab_imu_sigmas.gyro_bias_random_walk_noise_density);
  rovio_config->setGyroBiasCovarianceY(
      maplab_imu_sigmas.gyro_bias_random_walk_noise_density);
  rovio_config->setGyroBiasCovarianceZ(
      maplab_imu_sigmas.gyro_bias_random_walk_noise_density);
}

std::string getRovioConfigurationTemplateFile() {
  const char* rovio_config_template_path = getenv("ROVIO_CONFIG_DIR");
  CHECK_NE(rovio_config_template_path, static_cast<char*>(NULL))
      << "ROVIO_CONFIG_DIR environment variable is not set.\n"
      << "Source the Maplab environment from your workspace:\n"
      << "source devel/setup.bash";
  std::string rovio_config_template_file(rovio_config_template_path);
  rovio_config_template_file += "/rovio_default_config.info";
  CHECK(common::fileExists(rovio_config_template_file))
      << "ROVIO configuration template file does not exist: "
      << rovio_config_template_file;
  return rovio_config_template_file;
}
}  // namespace

rovio::RovioInterface* constructAndConfigureRovio(
    const aslam::NCamera& camera_calibration,
    const vi_map::ImuSigmas& imu_config) {
  // Translate the camera calibration to ROVIO format.
  size_t num_cameras = camera_calibration.getNumCameras();
  rovio::CameraCalibrationVector rovio_calibrations;
  convertAslamToRovioCamera(camera_calibration, &rovio_calibrations);

  // Load default ROVIO parameters from file and adapt where necessary.
  const std::string rovio_config_template_file =
      getRovioConfigurationTemplateFile();
  VLOG(1) << "Loading ROVIO configuration template: "
          << rovio_config_template_file;

  rovio::FilterConfiguration rovio_configuration(rovio_config_template_file);
  initFilterConfigurationFromGFLags(&rovio_configuration);
  setRovioImuSigmas(imu_config, &rovio_configuration);

  rovio::RovioInterface* rovio = nullptr;
  switch (num_cameras) {
    case 1:
      rovio = RovioBuilder<1>()(rovio_configuration, rovio_calibrations);
      break;
    case 2:
      rovio = RovioBuilder<2>()(rovio_configuration, rovio_calibrations);
      break;
    default:
      LOG(WARNING) << "Rovio support is only compiled for up to 2 cameras. "
                   << "Please adapt the code if you need more.";
  }
  return rovio;
}
}  // namespace rovioli
