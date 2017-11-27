#include "ceres-error-terms/ceres-signal-handler.h"

#include <signal.h>

#include <ceres/types.h>

namespace ceres_error_terms {

SignalHandlerCallback* signal_handler_link = nullptr;

SignalHandlerCallback::SignalHandlerCallback()
    : terminate_after_next_iteration_(false) {
  CHECK(signal_handler_link == nullptr)
      << "There is already a SignalHandlerCallback registered.";
  // Install a temporary SIGINT handler to abort the current optimization.
  signal_handler_link = this;
  sigaction(SIGINT, nullptr, &previous_signal_handler_);
  signal(SIGINT, static_cast<sig_t>(&signalHandler));
}

SignalHandlerCallback::~SignalHandlerCallback() {
  // Restore the original signal handler.
  signal(SIGINT, previous_signal_handler_.sa_handler);
  signal_handler_link = nullptr;
}

void SignalHandlerCallback::signalHandler(int signal) {
  CHECK_EQ(signal, SIGINT);
  CHECK(signal_handler_link != nullptr)
      << "No SignalHandlerCallback registered.";

  signal_handler_link->terminate_after_next_iteration_ = true;
  LOG(WARNING)
      << "User requested optimization termination after next update...";
}

ceres::CallbackReturnType SignalHandlerCallback::operator()(
    const ceres::IterationSummary&) {
  if (terminate_after_next_iteration_) {
    terminate_after_next_iteration_ = false;
    return ceres::SOLVER_TERMINATE_SUCCESSFULLY;
  }
  return ceres::SOLVER_CONTINUE;
}

}  // namespace ceres_error_terms
