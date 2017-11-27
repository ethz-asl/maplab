#include "visualization/debug-visualizer.h"

#include <atomic>
#include <memory>
#include <string>
#include <thread>

#include <gflags/gflags.h>
#include <glog/logging.h>

#include <map-manager/map-manager.h>

#include "visualization/viwls-graph-plotter.h"

DEFINE_bool(
    debug_visualizer_enabled, false,
    "Enables the default visualizer that plots the map defined by "
    "--debug_visualizer_map_key every --debug_visualizer_plot_every_s "
    "seconds.");

DEFINE_uint64(
    debug_visualizer_plot_every_ms, 1000u,
    "Interval at which the map defined by --debug_visualizer_map_key "
    "should be published. [ms]");

DEFINE_string(
    debug_visualizer_map_key, "",
    "Map key to the map that is visualized in a regular interval "
    "defined by --debug_visualizer_plot_every_s");

namespace visualization {

std::atomic<bool> DebugVisualizer::shutdown_requested_(false);
std::unique_ptr<std::thread> DebugVisualizer::visualizer_thread_(nullptr);

void DebugVisualizer::visualizeEveryInterval() {
  if (!FLAGS_debug_visualizer_enabled) {
    LOG(INFO) << "DebugVisualizer is disabled! Use --debug_visualizer_enabled "
                 "to enable.";
    return;
  } else {
    LOG(INFO) << "DebugVisualizer is enabled!";
  }
  CHECK(!FLAGS_debug_visualizer_map_key.empty());
  CHECK_GT(FLAGS_debug_visualizer_plot_every_ms, 0u);

  shutdown_requested_.store(false);

  visualization::ViwlsGraphRvizPlotter plotter;
  vi_map::VIMapManager manager;

  visualizer_thread_.reset(new std::thread([&]() {
    while (!shutdown_requested_.load()) {
      if (manager.hasMap(FLAGS_debug_visualizer_map_key)) {
        vi_map::VIMapManager::MapReadAccess map =
            manager.getMapReadAccess(FLAGS_debug_visualizer_map_key);
        plotter.visualizeMap(*map);
        std::this_thread::sleep_for(
            std::chrono::milliseconds(FLAGS_debug_visualizer_plot_every_ms));
      }
    }
  }));
}

void DebugVisualizer::visualizeNow(const std::string map_key) {
  if (!FLAGS_debug_visualizer_enabled) {
    LOG(INFO) << "DebugVisualizer is disabled! Use --debug_visualizer_enabled "
                 "to enable.";
    return;
  } else {
    LOG(INFO) << "DebugVisualizer is enabled!";
  }
  CHECK(!map_key.empty());

  visualization::ViwlsGraphRvizPlotter plotter;
  vi_map::VIMapManager manager;

  if (manager.hasMap(map_key)) {
    vi_map::VIMapManager::MapReadAccess map = manager.getMapReadAccess(map_key);
    plotter.visualizeMap(*map);
  }
}

void DebugVisualizer::shutdown() {
  CHECK(visualizer_thread_);
  shutdown_requested_.store(true);
}

void DebugVisualizer::shutdownAndWait() {
  CHECK(visualizer_thread_);
  shutdown_requested_.store(true);
  visualizer_thread_->join();
}
}  // namespace visualization
