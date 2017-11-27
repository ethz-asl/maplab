#ifndef CERES_ERROR_TERMS_CERES_SIGNAL_HANDLER_H_
#define CERES_ERROR_TERMS_CERES_SIGNAL_HANDLER_H_

#include <signal.h>

#include <ceres/iteration_callback.h>
#include <ceres/types.h>
#include <glog/logging.h>

namespace ceres {
struct IterationSummary;
}

namespace ceres_error_terms {
/// \class SignalHandlerCallback
/// \brief If this ceres iteration callback functor is registered, the
/// optimization can be
///        terminated using the SIGINT signal (usually CTRL+C on POSIX systems).
///        The current iteration will be finished, the state written back to the
///        output and the
///        optimization terminates successfully.
class SignalHandlerCallback : public ceres::IterationCallback {
 public:
  SignalHandlerCallback();
  virtual ~SignalHandlerCallback();

  static void signalHandler(int signal);
  virtual ceres::CallbackReturnType operator()(const ceres::IterationSummary&);

 private:
  bool terminate_after_next_iteration_;
  struct sigaction previous_signal_handler_;
};

}  // namespace ceres_error_terms

#endif  // CERES_ERROR_TERMS_CERES_SIGNAL_HANDLER_H_
