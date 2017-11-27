#ifndef MAPLAB_COMMON_SIGINT_BREAKER_H_
#define MAPLAB_COMMON_SIGINT_BREAKER_H_

namespace common {

class SigintBreaker {
 public:
  SigintBreaker();
  ~SigintBreaker();
  bool isBreakRequested() const;

 private:
  static void handler(int signal);

  static bool is_instantiated_;
  static bool is_sigint_raised_;
  void (*previous_handler_)(int);  // NOLINT
};

}  // namespace common

#endif  // MAPLAB_COMMON_SIGINT_BREAKER_H_
