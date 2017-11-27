#include "rovioli/map-builder-flow.h"

#include <functional>

#include <landmark-triangulation/landmark-triangulation.h>
#include <localization-summary-map/localization-summary-map-creation.h>
#include <localization-summary-map/localization-summary-map.h>
#include <maplab-common/file-logger.h>
#include <maplab-common/map-manager-config.h>
#include <mapping-workflows-plugin/localization-map-creation.h>
#include <vi-map-helpers/vi-map-landmark-quality-evaluation.h>
#include <vi-map-helpers/vi-map-manipulation.h>
#include <vi-map/vi-map-serialization.h>
#include <vi-map/vi-map.h>
#include <visualization/viwls-graph-plotter.h>

DEFINE_double(
    localization_map_keep_landmark_fraction, 0.0,
    "Fraction of landmarks to keep when creating a localization summary map.");
DECLARE_bool(rovioli_visualize_map);

namespace rovioli {

MapBuilderFlow::MapBuilderFlow(
    const std::shared_ptr<aslam::NCamera>& n_camera, vi_map::Imu::UniquePtr imu,
    const std::string& save_map_folder)
    : map_with_mutex_(aligned_shared<VIMapWithMutex>()),
      mapping_terminated_(false),
      stream_map_builder_(n_camera, std::move(imu), &map_with_mutex_->vi_map) {
  if (!save_map_folder.empty()) {
    VLOG(1) << "Set VIMap folder to: " << save_map_folder;
    map_with_mutex_->vi_map.setMapFolder(save_map_folder);
  }
}

void MapBuilderFlow::attachToMessageFlow(message_flow::MessageFlow* flow) {
  CHECK_NOTNULL(flow);
  static constexpr char kSubscriberNodeName[] = "MapBuilderFlow";
  std::function<void(const VIMapWithMutex::ConstPtr&)> map_publish_function =
      flow->registerPublisher<message_flow_topics::RAW_VIMAP>();
  CHECK(map_publish_function);
  flow->registerSubscriber<message_flow_topics::VIO_UPDATES>(
      kSubscriberNodeName, message_flow::DeliveryOptions(),
      [this, map_publish_function](const vio::VioUpdate::ConstPtr& vio_update) {
        CHECK(vio_update != nullptr);
        {
          std::lock_guard<std::mutex> lock(map_with_mutex_->mutex);
          if (mapping_terminated_) {
            return;
          }
          // WARNING: The tracker is updating the track information in the
          // current and previous frame; therefore the most recent VisualNFrame
          // added to the map might still be modified.
          constexpr bool kDeepCopyNFrame = false;
          stream_map_builder_.apply(*vio_update, kDeepCopyNFrame);
        }
        map_publish_function(map_with_mutex_);
      });

  std::function<void(const vio::VioUpdate::ConstPtr&)>
      vio_update_builder_publisher =
          flow->registerPublisher<message_flow_topics::VIO_UPDATES>();
  vio_update_builder_.registerVioUpdatePublishFunction(
      vio_update_builder_publisher);
  flow->registerSubscriber<message_flow_topics::TRACKED_NFRAMES_AND_IMU>(
      kSubscriberNodeName, message_flow::DeliveryOptions(),
      [this](
          const vio::SynchronizedNFrameImu::ConstPtr& synchronized_nframe_imu) {
        if (mapping_terminated_) {
          return;
        }
        vio_update_builder_.processSynchronizedNFrameImu(
            synchronized_nframe_imu);
      });
  flow->registerSubscriber<message_flow_topics::ROVIO_ESTIMATES>(
      kSubscriberNodeName, message_flow::DeliveryOptions(),
      [this](const RovioEstimate::ConstPtr& rovio_estimate) {
        if (mapping_terminated_) {
          return;
        }
        vio_update_builder_.processRovioEstimate(rovio_estimate);
      });
  flow->registerSubscriber<message_flow_topics::LOCALIZATION_RESULT>(
      kSubscriberNodeName, message_flow::DeliveryOptions(),
      std::bind(
          &VioUpdateBuilder::processLocalizationResult, &vio_update_builder_,
          std::placeholders::_1));
}

void MapBuilderFlow::saveMapAndOptionallyOptimize(
    const std::string& path, const bool overwrite_existing_map,
    const bool process_to_localization_map) {
  CHECK(!path.empty());
  CHECK(map_with_mutex_);

  std::lock_guard<std::mutex> lock(map_with_mutex_->mutex);
  mapping_terminated_ = true;

  // Early exit if the map is empty.
  if (map_with_mutex_->vi_map.numVertices() < 3u) {
    LOG(WARNING) << "Map is empty; nothing will be saved.";
    return;
  }

  visualization::ViwlsGraphRvizPlotter::UniquePtr plotter;
  if (FLAGS_rovioli_visualize_map) {
    plotter = aligned_unique<visualization::ViwlsGraphRvizPlotter>();
  }

  {
    VLOG(1) << "Initializing landmarks of created map.";
    // There should only be one mission in the map.
    vi_map::MissionIdList mission_ids;
    map_with_mutex_->vi_map.getAllMissionIds(&mission_ids);
    CHECK_EQ(mission_ids.size(), 1u);
    const vi_map::MissionId& id_of_first_mission = mission_ids.front();

    vi_map_helpers::VIMapManipulation manipulation(&map_with_mutex_->vi_map);
    manipulation.initializeLandmarksFromUnusedFeatureTracksOfMission(
        id_of_first_mission);

    landmark_triangulation::retriangulateLandmarksOfMission(
        id_of_first_mission, &map_with_mutex_->vi_map);
  }

  backend::SaveConfig save_config;
  save_config.overwrite_existing_files = overwrite_existing_map;
  vi_map::serialization::saveMapToFolder(
      path, save_config, &map_with_mutex_->vi_map);
  LOG(INFO) << "Raw VI-map saved to: " << path;

  if (process_to_localization_map) {
    LOG(INFO) << "Map is being processed into a localization map... "
              << "please wait.";

    map_sparsification::KeyframingHeuristicsOptions keyframe_options =
        map_sparsification::KeyframingHeuristicsOptions::initializeFromGFlags();
    constexpr bool kInitializeLandmarks = false;
    mapping_workflows_plugin::processVIMapToLocalizationMap(
        kInitializeLandmarks, keyframe_options, &map_with_mutex_->vi_map,
        plotter.get());

    // Create localization summary map that has pre-projected descriptors.
    std::unique_ptr<summary_map::LocalizationSummaryMap> localization_map(
        new summary_map::LocalizationSummaryMap);
    vi_map_helpers::evaluateLandmarkQuality(&map_with_mutex_->vi_map);

    if (FLAGS_localization_map_keep_landmark_fraction > 0.0) {
      summary_map::createLocalizationSummaryMapForSummarizedLandmarks(
          map_with_mutex_->vi_map,
          FLAGS_localization_map_keep_landmark_fraction,
          localization_map.get());
    } else {
      summary_map::createLocalizationSummaryMapForWellConstrainedLandmarks(
          map_with_mutex_->vi_map, localization_map.get());
    }

    std::string localization_map_path = path + "_localization";
    localization_map->saveToFolder(localization_map_path, save_config);
    LOG(INFO) << "Localization summary map saved to: " << localization_map_path;
  }
}
}  // namespace rovioli
