#ifndef REGISTRATION_TOOLBOX_TESTING_PLUGIN_REGISTRATION_TOOLBOX_TESTING_PLUGIN_H_
#define REGISTRATION_TOOLBOX_TESTING_PLUGIN_REGISTRATION_TOOLBOX_TESTING_PLUGIN_H_

#include <string>

#include <console-common/console-plugin-base-with-plotter.h>

DECLARE_string(map_mission);
DECLARE_string(map_mission_list);

namespace common {
class Console;
}  // namespace common

namespace visualization {
class ViwlsGraphRvizPlotter;
}  // namespace visualization

namespace registration_toolbox_testing {
class RegistrationToolboxTestingPlugin
    : public common::ConsolePluginBaseWithPlotter {
 public:
  RegistrationToolboxTestingPlugin(
      common::Console* console, visualization::ViwlsGraphRvizPlotter* plotter);

  virtual std::string getPluginId() const {
    return "registration_toolbox_testing";
  }

 private:
  int testRegistrationToolbox() const;
};
}  // namespace registration_toolbox_testing

#endif  // REGISTRATION_TOOLBOX_TESTING_PLUGIN_REGISTRATION_TOOLBOX_TESTING_PLUGIN_H_
