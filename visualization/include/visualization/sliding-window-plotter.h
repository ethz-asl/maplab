#ifndef VISUALIZATION_SLIDING_WINDOW_PLOTTER_H_
#define VISUALIZATION_SLIDING_WINDOW_PLOTTER_H_

#include <string>

#include <aslam/common/pose-types.h>
#include <vi-map/unique-id.h>
#include <vi-map/vi-map.h>

#include "visualization/color.h"
#include "visualization/viwls-graph-plotter.h"

namespace visualization {

class SlidingWindowPlotter {
 public:
  MAPLAB_POINTER_TYPEDEFS(SlidingWindowPlotter);

  explicit SlidingWindowPlotter(
      const ViwlsGraphRvizPlotter::Ptr& plotter,
      const size_t sliding_window_size)
      : kSlidingWindowColor(241, 204, 91),
        kOutOfWindowColor(130, 130, 130),
        kSlidingWindowSize(sliding_window_size),
        landmarks_topic_("sliding_window_landmarks"),
        plotter_(plotter) {
    CHECK(plotter_ != nullptr);
  }

  void setPlotter(const ViwlsGraphRvizPlotter::Ptr& plotter);

  void plotMissionWithSlidingWindow(
      const vi_map::VIMap& map, const pose_graph::VertexId& last_vertex);
  void plotMissionWithSlidingWindow(
      const vi_map::VIMap& map, const pose_graph::VertexIdSet& vertices);

 private:
  const visualization::Color kSlidingWindowColor;
  const visualization::Color kOutOfWindowColor;

  const size_t kSlidingWindowSize;
  pose_graph::VertexIdList last_sliding_window_vertices_;

  visualization::SphereVector sliding_window_landmarks_;
  visualization::SphereVector out_of_window_landmarks_;

  const std::string landmarks_topic_;

  ViwlsGraphRvizPlotter::Ptr plotter_;
};

}  // namespace visualization

#endif  // VISUALIZATION_SLIDING_WINDOW_PLOTTER_H_
