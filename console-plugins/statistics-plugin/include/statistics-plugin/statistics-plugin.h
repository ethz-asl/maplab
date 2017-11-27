#ifndef STATISTICS_PLUGIN_STATISTICS_PLUGIN_H_
#define STATISTICS_PLUGIN_STATISTICS_PLUGIN_H_

#include <string>

#include <console-common/console-plugin-base.h>
#include <console-common/console.h>

namespace statistics_plugin {

class StatisticsPlugin : public common::ConsolePluginBase {
 public:
  explicit StatisticsPlugin(common::Console* console);

  virtual std::string getPluginId() const override {
    return "statistics";
  }
};

}  // namespace statistics_plugin

#endif  // STATISTICS_PLUGIN_STATISTICS_PLUGIN_H_
