#include "maplab-common/sigint-breaker.h"

#include <csignal>

#include <glog/logging.h>

namespace common {

SigintBreaker::SigintBreaker()
    : previous_handler_(signal(SIGINT, &SigintBreaker::handler)) {
  CHECK(!is_instantiated_) << "Can't instantiate multiple SigintBreakers!";
  is_instantiated_ = true;
  is_sigint_raised_ = false;
}

SigintBreaker::~SigintBreaker() {
  CHECK_EQ(signal(SIGINT, previous_handler_), &SigintBreaker::handler);
  is_instantiated_ = false;
}

bool SigintBreaker::isBreakRequested() const {
  return is_sigint_raised_;
}

void SigintBreaker::handler(int signal) {
  CHECK_EQ(signal, SIGINT);
  is_sigint_raised_ = true;
}

bool SigintBreaker::is_instantiated_ = false;
bool SigintBreaker::is_sigint_raised_ = false;

}  // namespace common
