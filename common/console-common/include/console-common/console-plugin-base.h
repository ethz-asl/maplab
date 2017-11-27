#ifndef CONSOLE_COMMON_CONSOLE_PLUGIN_BASE_H_
#define CONSOLE_COMMON_CONSOLE_PLUGIN_BASE_H_

#include <functional>
#include <initializer_list>
#include <list>
#include <memory>
#include <string>
#include <vector>

#include <glog/logging.h>
#include <maplab-common/macros.h>

#include "console-common/command-registerer.h"

namespace visualization {
class ViwlsGraphRvizPlotter;
}  // namespace visualization

namespace common {

class Console;

class ConsolePluginBase {
  friend class Console;

 public:
  explicit ConsolePluginBase(Console* console)
      : console_(CHECK_NOTNULL(console)) {}

  virtual std::string getPluginId() const = 0;

  virtual ~ConsolePluginBase() {}

 protected:
  bool getSelectedMapKeyIfSet(std::string* selected_map_key) const;

  void addCommand(
      const std::initializer_list<std::string>& commands,
      const std::function<int()>& callback, const std::string& help_text,
      const Processing processing_model) {
    commands_.emplace_back(
        commands, callback, help_text, processing_model, getPluginId());
  }

  Console* console_;

 private:
  CommandRegisterer::Commands commands_;
};

// C-style function pointer typedefs necessary to interface with the C functions
// for loading dynamic libraries.
typedef ConsolePluginBase* (*PluginCreateFunction)(
    Console* console, visualization::ViwlsGraphRvizPlotter* plotter);
typedef void (*PluginDestroyFunction)(ConsolePluginBase* plugin);
typedef std::unique_ptr<ConsolePluginBase, PluginDestroyFunction>
    ConsolePluginPtr;

}  // namespace common

#define MAPLAB_CREATE_CONSOLE_PLUGIN(PluginName)                   \
  extern "C" common::ConsolePluginBase* createConsolePlugin(       \
      common::Console* console,                                    \
      visualization::ViwlsGraphRvizPlotter* /*plotter*/) {         \
    return new PluginName(console);                                \
  }                                                                \
                                                                   \
  extern "C" void destroyConsolePlugin(                            \
      common::ConsolePluginBase* console_plugin) {                 \
    delete console_plugin;                                         \
  }                                                                \
                                                                   \
  /* Dummy statement to enforce a `;` at the end of macro usage.*/ \
  static_assert(0 == 0, "")

#endif  // CONSOLE_COMMON_CONSOLE_PLUGIN_BASE_H_
