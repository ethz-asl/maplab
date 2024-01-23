#ifndef ASLAM_NULL_DISTORTION_H_
#define ASLAM_NULL_DISTORTION_H_

#include <Eigen/Core>
#include <aslam/cameras/distortion.h>
#include <aslam/common/crtp-clone.h>
#include <aslam/common/macros.h>
#include <glog/logging.h>

namespace aslam {

/// \class NullDistortion
/// \brief An implementation of the Null distortion model does nothing.
class NullDistortion : public aslam::Cloneable<Distortion, NullDistortion> {
 public:
  /** \brief Number of parameters used for this distortion model. */
  enum { kNumOfParams = 0 };

  enum { CLASS_SERIALIZATION_VERSION = 1 };
  ASLAM_POINTER_TYPEDEFS(NullDistortion);

  //////////////////////////////////////////////////////////////
  /// \name Constructors/destructors and operators
  /// @{

  /// \brief NullDistortion Ctor.
  NullDistortion() : Base(Eigen::VectorXd(), Distortion::Type::kNoDistortion) {}

  /// \brief Convenience function to print the state using streams.
  friend std::ostream& operator<<(
      std::ostream& out, const NullDistortion& distortion);

 public:
  /// Copy constructor for clone operation.
  NullDistortion(const NullDistortion&) = default;
  void operator=(const NullDistortion&) = delete;

  /// @}

  //////////////////////////////////////////////////////////////
  /// \name Distort methods: applies the distortion model to a point.
  /// @{
 public:
  /// \brief Apply distortion to a point in the normalized image plane using
  /// provided distortion
  ///        coefficients. External distortion coefficients can be specified
  ///        using this function. (Ignores the internally stored parameters.
  /// @param[in]     dist_coeffs  Vector containing the coefficients for the
  /// distortion model.
  ///                             NOTE: If nullptr, use internal distortion
  ///                             parameters.
  /// @param[in,out] point        The point in the normalized image plane. After
  /// the function,
  ///                             this point is distorted.
  /// @param[out]    out_jacobian The Jacobian of the distortion function with
  /// respect to small
  ///                             changes in the input point. If NULL is passed,
  ///                             the Jacobian calculation is skipped.
  virtual void distortUsingExternalCoefficients(
      const Eigen::VectorXd* /* dist_coeffs */, Eigen::Vector2d* /* point */,
      Eigen::Matrix2d* out_jacobian) const {
    if (out_jacobian) {
      out_jacobian->setIdentity();
    }
  }

  /// \brief Template version of the distortExternalCoeffs function.
  /// @param[in]  dist_coeffs Vector containing the coefficients for the
  /// distortion model.
  /// @param[in]  point       The point in the normalized image plane. After the
  /// function, this
  ///                         point is distorted.
  /// @param[out] out_point   The distorted point.
  template <typename ScalarType, typename MDistortion>
  void distortUsingExternalCoefficients(
      const Eigen::MatrixBase<MDistortion>& /* dist_coeffs */,
      const Eigen::Matrix<ScalarType, 2, 1>& point,
      Eigen::Matrix<ScalarType, 2, 1>* out_point) const {
    CHECK_NOTNULL(out_point);
    *out_point = point;
  }

  /// \brief Apply distortion to the point and provide the Jacobian of the
  /// distortion with respect
  ///        to small changes in the distortion parameters.
  /// @param[in]  dist_coeffs  Vector containing the coefficients for the
  /// distortion model.
  ///                          NOTE: If nullptr, use internal distortion
  ///                          parameters.
  /// @param[in]  point        The point in the normalized image plane.
  /// @param[out] out_jacobian The Jacobian of the distortion with respect to
  /// small changes in
  ///                          the distortion parameters.
  virtual void distortParameterJacobian(
      const Eigen::VectorXd* /* dist_coeffs */,
      const Eigen::Vector2d& /* point */,
      Eigen::Matrix<double, 2, Eigen::Dynamic>* out_jacobian) const {
    if (out_jacobian) {
      out_jacobian->resize(2, 0);
    }
  }

  /// @}

  //////////////////////////////////////////////////////////////
  /// \name Undistort methods: Removes the modeled distortion effects from a
  /// point.
  /// @{

  /// \brief Apply undistortion to recover a point in the normalized image plane
  /// using provided
  ///        distortion coefficients. External distortion coefficients can be
  ///        specified using this function. Ignores the internally  stored
  ///        parameters.
  /// @param[in]      dist_coeffs  Vector containing the coefficients for the
  /// distortion model.
  /// @param[in,out]  point        The distorted point. After the function, this
  /// point is in the
  ///                              normalized image plane.
  virtual void undistortUsingExternalCoefficients(
      const Eigen::VectorXd& /*dist_coeffs*/,
      Eigen::Vector2d* /*point*/) const {}

  /// @}

  //////////////////////////////////////////////////////////////
  /// \name Methods to support unit testing.
  /// @{

  /// \brief Create a test distortion object for unit testing.
  static NullDistortion::UniquePtr createTestDistortion() {
    return NullDistortion::UniquePtr(new NullDistortion());
  }

  /// @}

  /////////////////////////////////////////////////////////////////////////////
  /// \name Methods to set/get distortion parameters
  /// @{

  /// Static function that checks whether the given intrinsic parameters are
  /// valid for this model.
  static bool areParametersValid(const Eigen::VectorXd& /*parameters*/) {
    // As the parameters do not have an impact, all parameters are valid.
    return true;
  }

  /// \brief Check the validity of distortion parameters.
  /// @param[in] dist_coeffs Vector containing the coefficients.
  ///            Parameters will NOT be stored.
  /// @return If the distortion parameters are valid.
  virtual bool distortionParametersValid(
      const Eigen::VectorXd& /*dist_coeffs*/) const {
    // As the parameters do not have an impact, all parameters are valid.
    return true;
  }

  /// \brief Returns the number of parameters used in this distortion model.
  inline static constexpr size_t parameterCount() {
    return kNumOfParams;
  }

  /// \brief Returns the number of parameters used in the distortion model.
  ///        NOTE: Use the constexpr function parameterCount if you know the
  ///        exact distortion type.
  virtual int getParameterSize() const {
    return kNumOfParams;
  }

  /// \brief Print the internal parameters of the distortion in a human-readable
  /// form Print to the ostream that is passed in. The text is extra text used
  /// by the calling function to distinguish cameras.
  virtual void printParameters(
      std::ostream& out, const std::string& text) const {
    out << text << std::endl;
    out << "Distortion: (NullDistortion) " << std::endl;
  }
  /// @}
};

inline std::ostream& operator<<(
    std::ostream& out, const NullDistortion& distortion) {
  distortion.printParameters(out, std::string(""));
  return out;
}

}  // namespace aslam

#endif  // ASLAM_NULL_DISTORTION_H_
