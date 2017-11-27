#include "statistics-plugin/statistics-plugin.h"

#include <iostream>  //NOLINT

#include <aslam/common/statistics/statistics.h>
#include <aslam/common/timer.h>
#include <console-common/console.h>

namespace statistics_plugin {

StatisticsPlugin::StatisticsPlugin(common::Console* console)
    : common::ConsolePluginBase(console) {
  addCommand(
      {"timing"},
      []() -> int {
        timing::Timing::Print(std::cout);
        return common::kSuccess;
      },
      "Print timing.", common::Processing::Sync);

  addCommand(
      {"statistics", "stat"},
      []() -> int {
        statistics::Statistics::Print(std::cout);
        return common::kSuccess;
      },
      "Print statistics.", common::Processing::Sync);
}

}  // namespace statistics_plugin

MAPLAB_CREATE_CONSOLE_PLUGIN(statistics_plugin::StatisticsPlugin);
