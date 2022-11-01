#ifndef CERES_ERROR_TERMS_SWITCH_PRIOR_ERROR_TERM_INL_H_
#define CERES_ERROR_TERMS_SWITCH_PRIOR_ERROR_TERM_INL_H_

namespace ceres_error_terms {
template <typename T>
bool SwitchPriorErrorTerm::operator()(
    const T* const switch_variable, T* residual) const {
  CHECK_GE(*switch_variable, static_cast<T>(0));
  CHECK_LE(*switch_variable, static_cast<T>(1));

  *residual = (static_cast<T>(prior_) - (*switch_variable)) *
              static_cast<T>(sqrt_information_factor_);
  return true;
}
}  // namespace ceres_error_terms

#endif  // CERES_ERROR_TERMS_SWITCH_PRIOR_ERROR_TERM_INL_H_
