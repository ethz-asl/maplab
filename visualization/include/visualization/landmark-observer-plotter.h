#ifndef VISUALIZATION_LANDMARK_OBSERVER_PLOTTER_H_
#define VISUALIZATION_LANDMARK_OBSERVER_PLOTTER_H_

#include <string>

#include <aslam/common/pose-types.h>
#include <geometry_msgs/PointStamped.h>
#include <vi-map-helpers/vi-map-nearest-neighbor-lookup.h>
#include <vi-map/unique-id.h>
#include <vi-map/vi-map.h>

namespace visualization {

class LandmarkObserverPlotter {
 public:
  static const std::string kClickedPointTopic;
  static const std::string kLandmarkObserverRaysTopic;

  LandmarkObserverPlotter() = delete;
  explicit LandmarkObserverPlotter(const vi_map::VIMap& vi_map);
  virtual ~LandmarkObserverPlotter() {}

  // Visualizes the observer rays for all clicked landmarks. This function is
  // blocking and can be stopped by pressing Ctrl+C (sigint handler).
  void visualizeClickedLandmarks() const;

 private:
  void clickedPointCallback(
      const geometry_msgs::PointStamped::ConstPtr& clicked_point_msg) const;

  const vi_map::VIMap& vi_map_;
  const vi_map_helpers::VIMapNearestNeighborLookupLandmarkId landmark_database_;
};

}  // namespace visualization

#endif  // VISUALIZATION_LANDMARK_OBSERVER_PLOTTER_H_
