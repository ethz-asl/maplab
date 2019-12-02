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
  MapLabConsole(
      const std::string& console_name, int argc, char** argv,
      const bool enable_plotter = true);
  ~MapLabConsole();

  // Create a new console by installing the the same plugins as other console.
  // The intended use-case is having multiple consoles running in different
  // threads, processing a list of commands. Auto completion will be disabled,
  // since there can only be one auto completion callback per process and we
  // will have multiple consoles.
  MapLabConsole(
      const MapLabConsole& other_console, const std::string& new_console_name,
      const bool enable_plotter = true);

 private:
  void discoverAndInstallPlugins(int argc, char** argv);

  bool enable_plotter_;
  std::vector<void*> plugin_handles_;
  visualization::ViwlsGraphRvizPlotter::UniquePtr plotter_;
};

}  // namespace maplab
#endif  // MAPLAB_CONSOLE_MAPLAB_CONSOLE_H_
