#ifndef MAPLAB_CONSOLE_MAPLAB_CONSOLE_H_
#define MAPLAB_CONSOLE_MAPLAB_CONSOLE_H_

#include <string>
#include <vector>

#include <console-common/console.h>
#include <visualization/viwls-graph-plotter.h>

namespace visualization {
class ViwlsGraphRvizPlotter;
}  // namespace visualization

namespace maplab {

class MapLabConsole : public common::Console {
 public:
  // This takes in argc and argv as gflags initialization is delayed until after
  // all plugins have been loaded (from within the constructor).
  MapLabConsole(const std::string& console_name, int argc, char** argv);
  ~MapLabConsole();

 private:
  void discoverAndInstallPlugins(int argc, char** argv);

  std::vector<void*> plugin_handles_;
  visualization::ViwlsGraphRvizPlotter::UniquePtr plotter_;
};

}  // namespace maplab
#endif  // MAPLAB_CONSOLE_MAPLAB_CONSOLE_H_
