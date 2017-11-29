#include <memory>
#include <string>

#include <glog/logging.h>

#include <console-common/console-plugin-base.h>
#include <console-common/console.h>
#include <maplab-common/glog-helpers.h>

#include "maplab-console/maplab-console.h"

static constexpr char kConsoleName[] = "maplab";

int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);
  google::InstallFailureSignalHandler();
  google::InstallFailureFunction(&common::glogFailFunctionWithConsoleRecovery);
  FLAGS_alsologtostderr = true;
  FLAGS_colorlogtostderr = true;

  // Ignore the SIGINT signal.
  signal(SIGINT, [](int) -> void {});

  maplab::MapLabConsole console(kConsoleName, argc, argv);
  console.RunCommandPrompt();

  return 0;
}
