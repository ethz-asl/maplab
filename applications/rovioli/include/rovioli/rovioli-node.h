#ifndef ROVIOLI_ROVIOLI_NODE_H_
#define ROVIOLI_ROVIOLI_NODE_H_
#include <memory>

#include <atomic>
#include <string>

#include <message-flow/message-flow.h>
#include <sensors/imu.h>

#include "rovioli/data-publisher-flow.h"
#include "rovioli/datasource-flow.h"
#include "rovioli/feature-tracking-flow.h"
#include "rovioli/imu-camera-synchronizer-flow.h"
#include "rovioli/localizer-flow.h"
#include "rovioli/map-builder-flow.h"
#include "rovioli/rovio-flow.h"
#include "rovioli/synced-nframe-throttler-flow.h"

namespace rovioli {
class RovioliNode final {
 public:
  RovioliNode(
      const aslam::NCamera::Ptr& camera_system,
      vi_map::Imu::UniquePtr maplab_imu_sensor,
      const vi_map::ImuSigmas& rovio_imu_sigmas,
      const std::string& save_map_folder,
      const summary_map::LocalizationSummaryMap* const localization_map,
      message_flow::MessageFlow* flow);
  ~RovioliNode();

  void start();
  void shutdown();

  // Save the map to disk. Optionally keyframe, optimize and summarize the map.
  void saveMapAndOptionallyOptimize(
      const std::string& path, const bool overwrite_existing_map,
      const bool process_to_localization_map);

  std::atomic<bool>& isDataSourceExhausted();

 private:
  std::unique_ptr<DataSourceFlow> datasource_flow_;
  std::unique_ptr<RovioFlow> rovio_flow_;
  std::unique_ptr<LocalizerFlow> localizer_flow_;
  std::unique_ptr<ImuCameraSynchronizerFlow> synchronizer_flow_;
  std::unique_ptr<FeatureTrackingFlow> tracker_flow_;
  std::unique_ptr<SyncedNFrameThrottlerFlow> throttler_flow_;
  std::unique_ptr<MapBuilderFlow> map_builder_flow_;
  std::unique_ptr<DataPublisherFlow> data_publisher_flow_;

  // Set to true once the data-source has played back all its data. Will never
  // be true for infinite data-sources (live-data).
  std::atomic<bool> is_datasource_exhausted_;
};
}  // namespace rovioli
#endif  // ROVIOLI_ROVIOLI_NODE_H_
