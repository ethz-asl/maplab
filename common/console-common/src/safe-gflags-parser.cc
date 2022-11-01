#include "console-common/safe-gflags-parser.h"

#include <gflags/gflags.h>

namespace GFLAGS_NAMESPACE {
extern void (*gflags_exitfunc)(int);
}  // namespace GFLAGS_NAMESPACE

namespace common {

namespace {

bool gflags_parsing_succeeded;

void handleGFlagsError(int /*error_code*/) {
  gflags_parsing_succeeded = false;
}

}  // namespace

bool parseCommandLineFlagsSafe(int argc, char** argv) {
  // Set gflags exit function to a custom function so that the console
  // doesn't crash on parsing error. To make sure that we don't break any
  // other gflags-related functionality, we reverse to old exit function
  // afterwards.
  gflags_parsing_succeeded = true;
  void (*old_gflags_error_handler)(int) = GFLAGS_NAMESPACE::gflags_exitfunc;
  GFLAGS_NAMESPACE::gflags_exitfunc = &handleGFlagsError;
  google::ParseCommandLineFlags(&argc, &argv, false);
  GFLAGS_NAMESPACE::gflags_exitfunc = old_gflags_error_handler;
  return gflags_parsing_succeeded;
}

}  // namespace common
