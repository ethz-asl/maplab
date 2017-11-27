#ifndef MAP_OPTIMIZATION_CALLBACKS_H_
#define MAP_OPTIMIZATION_CALLBACKS_H_

#include <memory>
#include <vector>

#include <ceres-error-terms/ceres-signal-handler.h>
#include <ceres/iteration_callback.h>
#include <map-optimization/optimization-state-buffer.h>
#include <visualization/viwls-graph-plotter.h>

namespace map_optimization {

class CopyBackToMapCallback : public ceres::IterationCallback {
 public:
  CopyBackToMapCallback(
      size_t copy_every_n, const OptimizationStateBuffer& buffer,
      vi_map::VIMap* map)
      : copy_every_n_(copy_every_n),
        buffer_(buffer),
        map_(map),
        iteration_(0u) {
    CHECK_NOTNULL(map_);
    CHECK_GT(copy_every_n_, 0u);
  }

  ceres::CallbackReturnType operator()(
      const ceres::IterationSummary& /*summary*/) {
    if (iteration_ % copy_every_n_ == 0) {
      buffer_.copyAllStatesBackToMap(map_);
    }
    ++iteration_;
    return ceres::SOLVER_CONTINUE;
  }

 private:
  const size_t copy_every_n_;
  const OptimizationStateBuffer& buffer_;
  vi_map::VIMap* const map_;

  // Count iterations locally as outlier rejection optimization is restarting
  // the ceres solver multiple times and zeroing the iteration count.
  size_t iteration_;
};

class VisualizationCallback : public ceres::IterationCallback {
 public:
  VisualizationCallback(
      size_t visualize_every_n,
      const visualization::ViwlsGraphRvizPlotter& plotter,
      const vi_map::VIMap& map)
      : visualize_every_n_(visualize_every_n),
        map_(map),
        plotter_(plotter),
        iteration_(0u) {
    CHECK_GT(visualize_every_n_, 0u);
  }

  ceres::CallbackReturnType operator()(
      const ceres::IterationSummary& /*summary*/) {
    if (iteration_ % visualize_every_n_ == 0u) {
      plotter_.visualizeMap(map_);
    }
    ++iteration_;
    return ceres::SOLVER_CONTINUE;
  }

 private:
  const size_t visualize_every_n_;
  const vi_map::VIMap& map_;
  const visualization::ViwlsGraphRvizPlotter& plotter_;

  // Count iterations locally as outlier rejection optimization is restarting
  // the ceres solver multiple times and zeroing the iteration count.
  size_t iteration_;
};

inline void appendSignalHandlerCallback(
    std::vector<std::shared_ptr<ceres::IterationCallback>>* callbacks) {
  CHECK_NOTNULL(callbacks);
  callbacks->emplace_back(new ceres_error_terms::SignalHandlerCallback());
}

inline void appendVisualizationCallbacks(
    size_t visualize_every_n, const OptimizationStateBuffer& buffer,
    const visualization::ViwlsGraphRvizPlotter& plotter, vi_map::VIMap* map,
    std::vector<std::shared_ptr<ceres::IterationCallback>>* callbacks) {
  CHECK_NOTNULL(map);
  CHECK_NOTNULL(callbacks);

  if (visualize_every_n > 0u) {
    // The order is important. CopyBackToMapCallback has to be added before
    // VisualizationCallback so that we are visualizing the current state.
    callbacks->emplace_back(
        new CopyBackToMapCallback(visualize_every_n, buffer, map));
    callbacks->emplace_back(
        new VisualizationCallback(visualize_every_n, plotter, *map));
  }
}

}  // namespace map_optimization

#endif  // MAP_OPTIMIZATION_CALLBACKS_H_
