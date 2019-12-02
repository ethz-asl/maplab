#include "ceres-error-terms/ceres-signal-handler.h"

#include <signal.h>
#include <vector>

#include <ceres/types.h>

namespace ceres_error_terms {

std::vector<SignalHandlerCallback*> signal_handler_links;

SignalHandlerCallback::SignalHandlerCallback()
    : terminate_after_next_iteration_(false) {
  // Install a temporary SIGINT handler to abort the current optimization.
  signal_handler_links.push_back(this);
  sigaction(SIGINT, nullptr, &previous_signal_handler_);
  signal(SIGINT, static_cast<sig_t>(&signalHandler));
}

SignalHandlerCallback::~SignalHandlerCallback() {
  // Restore the original signal handler.
  signal(SIGINT, previous_signal_handler_.sa_handler);

  // Remove the pointer to this signal hander from the global handler links.
  auto it = signal_handler_links.begin();
  while (it != signal_handler_links.end()) {
    if (*it == this) {
      it = signal_handler_links.erase(it);
    } else {
      ++it;
    }
  }
}

void SignalHandlerCallback::signalHandler(int signal) {
  CHECK_EQ(signal, SIGINT);
  CHECK(!signal_handler_links.empty())
      << "Signal was received, but no signal handler was registered!";

  for (SignalHandlerCallback* callback : signal_handler_links) {
    callback->terminate_after_next_iteration_ = true;
  }
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
