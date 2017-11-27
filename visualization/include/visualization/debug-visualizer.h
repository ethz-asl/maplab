#ifndef VISUALIZATION_DEBUG_VISUALIZER_H_
#define VISUALIZATION_DEBUG_VISUALIZER_H_

#include <atomic>
#include <memory>
#include <string>
#include <thread>

namespace visualization {

class DebugVisualizer {
 public:
  // Start visualizing a map in regular intervals. This call is non-blocking.
  // Use the following gflags to configure the visualizer:
  //  --debug_visualizer_enable
  //  --debug_visualizer_map_key
  //  --debug_visualizer_plot_every_ms
  // Call shutdown() or shutdownAndWait() to stop the visualization.
  static void visualizeEveryInterval();

  // Visualize the map defined by map_key once.
  // Doesn't do anything if --debug_visualizer_enable=false.
  // Does not require a shutdown.
  static void visualizeNow(const std::string map_key);

  // Shutdown visualizer. This call is blocking, i.e. waits for the visualizer
  // to shutdown.
  static void shutdownAndWait();

  // Shutdown visualizer. This call is non-blocking, i.e. it doesn't wait for
  // the visualizer to shutdown.
  static void shutdown();

 private:
  static std::atomic<bool> shutdown_requested_;
  static std::unique_ptr<std::thread> visualizer_thread_;
};

}  // namespace visualization

#endif  // VISUALIZATION_DEBUG_VISUALIZER_H_
