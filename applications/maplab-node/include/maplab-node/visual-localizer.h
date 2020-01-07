#ifndef MAPLAB_NODE_VISUAL_LOCALIZER_H_
#define MAPLAB_NODE_VISUAL_LOCALIZER_H_

#include <memory>
#include <vector>

#include <localization-summary-map/localization-summary-map-queries.h>
#include <localization-summary-map/localization-summary-map.h>
#include <loop-closure-handler/loop-detector-node.h>
#include <maplab-common/localization-result.h>
#include <maplab-common/macros.h>
#include <vio-common/vio-types.h>

namespace maplab {

class VisualLocalizer {
 public:
  typedef common::LocalizationMode LocalizationMode;

  VisualLocalizer() = delete;
  MAPLAB_POINTER_TYPEDEFS(VisualLocalizer);
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  VisualLocalizer(
      const vi_map::SensorManager& sensor_manager,
      std::unique_ptr<summary_map::LocalizationSummaryMap>
          localization_summary_map,
      const bool visualize_localization);

  VisualLocalizer(
      const vi_map::SensorManager& sensor_manager,
      const bool visualize_localization);

  void setLocalizationMap(
      std::unique_ptr<summary_map::LocalizationSummaryMap> localization_map);

  LocalizationMode getCurrentLocalizationMode() const;

  bool localizeNFrame(
      const aslam::VisualNFrame::ConstPtr& nframe,
      vio::LocalizationResult* localization_result) const;

 private:
  bool localizeNFrameGlobal(
      const aslam::VisualNFrame::ConstPtr& nframe,
      vio::LocalizationResult* localization_result) const;
  bool localizeNFrameMapTracking(
      const aslam::VisualNFrame::ConstPtr& nframe,
      vio::LocalizationResult* localization_result) const;

  const vi_map::SensorManager& sensor_manager_;

  LocalizationMode current_localization_mode_;
  loop_detector_node::LoopDetectorNode::UniquePtr global_loop_detector_;

  std::unique_ptr<summary_map::LocalizationSummaryMap> localization_map_;
  std::unique_ptr<summary_map::SummaryMapCachedLookups> map_cached_lookup_;
};

}  // namespace maplab

#endif  // MAPLAB_NODE_VISUAL_LOCALIZER_H_
