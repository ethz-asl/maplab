#ifndef CERES_ERROR_TERMS_SWITCH_PRIOR_ERROR_TERM_H_
#define CERES_ERROR_TERMS_SWITCH_PRIOR_ERROR_TERM_H_

namespace ceres_error_terms {
namespace internal {
template <bool kAssertValidVariableBounds = true>
class SwitchPriorErrorTermImpl {
 public:
  SwitchPriorErrorTermImpl(double prior, double variance) : prior_(prior) {
    CHECK_GT(variance, 0.0);
    sqrt_information_factor_ = std::sqrt(1.0 / variance);
  }

  template <typename T>
  bool operator()(const T* const switch_variable, T* residual) const;

  static constexpr int residualBlockSize = 1;
  static constexpr int switchVariableBlockSize = 1;

 private:
  double prior_;
  double sqrt_information_factor_;
};
}  // namespace internal

using SwitchPriorErrorTerm =
    internal::SwitchPriorErrorTermImpl</*kAssertValidVariableBounds=*/true>;
using SwitchPriorErrorTermLegacy =
    internal::SwitchPriorErrorTermImpl</*kAssertValidVariableBounds=*/false>;

}  // namespace ceres_error_terms
#include "./ceres-error-terms/switch-prior-error-term-inl.h"
#endif  // CERES_ERROR_TERMS_SWITCH_PRIOR_ERROR_TERM_H_
