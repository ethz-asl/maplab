#ifndef ROVIOLI_ROVIO_FACTORY_H_
#define ROVIOLI_ROVIO_FACTORY_H_

#include <aslam/cameras/ncamera.h>
#include <rovio/RovioInterfaceBuilder.hpp>
#include <sensors/imu.h>

namespace rovioli {
rovio::RovioInterface* constructAndConfigureRovio(
    const aslam::NCamera& camera_calibration,
    const vi_map::ImuSigmas& imu_sigmas);
}  // namespace rovioli
#endif  // ROVIOLI_ROVIO_FACTORY_H_
