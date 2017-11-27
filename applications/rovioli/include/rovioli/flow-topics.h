#ifndef ROVIOLI_FLOW_TOPICS_H_
#define ROVIOLI_FLOW_TOPICS_H_
#include <message-flow/message-topic-registration.h>
#include <vi-map/vi-map.h>
#include <vio-common/vio-types.h>
#include <vio-common/vio-update.h>

#include "rovioli/rovio-estimate.h"
#include "rovioli/vi-map-with-mutex.h"

// TODO(schneith): All message should be ConstPtr.

// Raw sensor data.
MESSAGE_FLOW_TOPIC(IMAGE_MEASUREMENTS, vio::ImageMeasurement::Ptr);
MESSAGE_FLOW_TOPIC(IMU_MEASUREMENTS, vio::ImuMeasurement::Ptr);

// Synchronized images from multiple cameras and IMU data.
MESSAGE_FLOW_TOPIC(SYNCED_NFRAMES_AND_IMU, vio::SynchronizedNFrameImu::Ptr);

MESSAGE_FLOW_TOPIC(
    TRACKED_NFRAMES_AND_IMU, vio::SynchronizedNFrameImu::ConstPtr);

MESSAGE_FLOW_TOPIC(
    THROTTLED_TRACKED_NFRAMES_AND_IMU, vio::SynchronizedNFrameImu::ConstPtr);

// Output of the localizer.
MESSAGE_FLOW_TOPIC(LOCALIZATION_RESULT, vio::LocalizationResult::ConstPtr);

// Raw estimate of the VINS.
MESSAGE_FLOW_TOPIC(VIO_UPDATES, vio::VioUpdate::ConstPtr);

// Raw estimate output of ROVIO.
MESSAGE_FLOW_TOPIC(ROVIO_ESTIMATES, rovioli::RovioEstimate::ConstPtr);

// Resulting map.
MESSAGE_FLOW_TOPIC(RAW_VIMAP, rovioli::VIMapWithMutex::ConstPtr);

// All data input subscribers are put in an exclusivity group such that the
// delivery ordering for all messages (cam, imu, localization) are
// corresponding to the publishing order and no sensor can be left behind.
constexpr int kExclusivityGroupIdRovioSensorSubscribers = 0;

#endif  // ROVIOLI_FLOW_TOPICS_H_
