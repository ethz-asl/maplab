#ifndef CONSOLE_COMMON_CONSOLE_PLUGIN_BASE_WITH_PLOTTER_H_
#define CONSOLE_COMMON_CONSOLE_PLUGIN_BASE_WITH_PLOTTER_H_

#include <string>

#include <glog/logging.h>

#include "console-common/console-plugin-base.h"

namespace visualization {
class ViwlsGraphRvizPlotter;
}  // namespace visualization

namespace common {

class Console;

class ConsolePluginBaseWithPlotter : public ConsolePluginBase {
 public:
  ConsolePluginBaseWithPlotter(
      Console* console, visualization::ViwlsGraphRvizPlotter* plotter)
      : ConsolePluginBase(CHECK_NOTNULL(console)),
                          plotter_(CHECK_NOTNULL(plotter)) {}

  virtual std::string getPluginId() const override = 0;

 protected:
  visualization::ViwlsGraphRvizPlotter* plotter_;
};

}  // namespace common

#define MAPLAB_CREATE_CONSOLE_PLUGIN_WITH_PLOTTER(PluginName)      \
  extern "C" common::ConsolePluginBase* createConsolePlugin(       \
      common::Console* console,                                    \
      visualization::ViwlsGraphRvizPlotter* plotter) {             \
    return new PluginName(console, plotter);                       \
  }                                                                \
                                                                   \
  extern "C" void destroyConsolePlugin(                            \
      common::ConsolePluginBase* console_plugin) {                 \
    delete console_plugin;                                         \
  }                                                                \
                                                                   \
  /* Dummy statement to enforce a `;` at the end of macro usage.*/ \
  static_assert(0 == 0, "")

#endif  // CONSOLE_COMMON_CONSOLE_PLUGIN_BASE_WITH_PLOTTER_H_
