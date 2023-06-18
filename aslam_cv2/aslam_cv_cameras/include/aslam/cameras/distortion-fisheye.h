#ifndef ASLAM_FISHEYE_DISTORTION_H_
#define ASLAM_FISHEYE_DISTORTION_H_

#include <Eigen/Core>
#include <glog/logging.h>

#include <aslam/common/crtp-clone.h>
#include <aslam/cameras/distortion.h>
#include <aslam/common/macros.h>

namespace aslam {

/// \class FisheyeDistortion
/// \brief An implementation of the fisheye distortion model for pinhole cameras.
class FisheyeDistortion : public aslam::Cloneable<Distortion, FisheyeDistortion> {
 public:
  /** \brief Number of parameters used for this distortion model. */
  enum { kNumOfParams = 1 };

  enum { CLASS_SERIALIZATION_VERSION = 1 };
  ASLAM_POINTER_TYPEDEFS(FisheyeDistortion);

  //////////////////////////////////////////////////////////////
  /// \name Constructors/destructors and operators
  /// @{

  /// \brief FisheyeDistortion Ctor.
  /// @param[in] distortionParams Vector containing the distortion parameter. (dim=1: w)
  explicit FisheyeDistortion(const Eigen::VectorXd& distortionParams);

  /// \brief Convenience function to print the state using streams.
  friend std::ostream& operator<<(std::ostream& out, const FisheyeDistortion& distortion);

 public:
  /// Copy constructor for clone operation.
  FisheyeDistortion(const FisheyeDistortion&) = default;
  void operator=(const FisheyeDistortion&) = delete;

  /// @}

  //////////////////////////////////////////////////////////////
  /// \name Distort methods: applies the distortion model to a point.
  /// @{
 public:
  /// \brief Apply distortion to a point in the normalized image plane using provided distortion
  ///        coefficients. External distortion coefficients can be specified using this function.
  ///        (Ignores the internally stored parameters.
  /// @param[in]     dist_coeffs  Vector containing the coefficients for the distortion model.
  ///                             NOTE: If nullptr, use internal distortion parameters.
  /// @param[in,out] point        The point in the normalized image plane. After the function,
  ///                             this point is distorted.
  /// @param[out]    out_jacobian The Jacobian of the distortion function with respect to small
  ///                             changes in the input point. If NULL is passed, the Jacobian
  ///                             calculation is skipped.
  virtual void distortUsingExternalCoefficients(const Eigen::VectorXd* dist_coeffs,
                                                Eigen::Vector2d* point,
                                                Eigen::Matrix2d* out_jacobian) const;

  /// \brief Template version of the distortExternalCoeffs function.
  /// @param[in]  dist_coeffs Vector containing the coefficients for the distortion model.
  /// @param[in]  point       The point in the normalized image plane. After the function, this
  ///                         point is distorted.
  /// @param[out] out_point   The distorted point.
  template <typename ScalarType, typename MDistortion>
  void distortUsingExternalCoefficients(const Eigen::MatrixBase<MDistortion>& dist_coeffs,
                                        const Eigen::Matrix<ScalarType, 2, 1>& point,
                                        Eigen::Matrix<ScalarType, 2, 1>* out_point) const;

  /// \brief Apply distortion to the point and provide the Jacobian of the distortion with respect
  ///        to small changes in the distortion parameters.
  /// @param[in]  dist_coeffs  Vector containing the coefficients for the distortion model.
  ///                          NOTE: If nullptr, use internal distortion parameters.
  /// @param[in]  point        The point in the normalized image plane.
  /// @param[out] out_jacobian The Jacobian of the distortion with respect to small changes in
  ///                          the distortion parameters.
  virtual void distortParameterJacobian(const Eigen::VectorXd* dist_coeffs,
                                        const Eigen::Vector2d& point,
                                        Eigen::Matrix<double, 2, Eigen::Dynamic>* out_jacobian) const;

  /// @}

  //////////////////////////////////////////////////////////////
  /// \name Undistort methods: Removes the modeled distortion effects from a point.
  /// @{

  /// \brief Apply undistortion to recover a point in the normalized image plane using provided
  ///        distortion coefficients. External distortion coefficients can be specified using this
  ///        function. Ignores the internally  stored parameters.
  /// @param[in]      dist_coeffs  Vector containing the coefficients for the distortion model.
  /// @param[in,out]  point        The distorted point. After the function, this point is in the
  ///                              normalized image plane.
  virtual void undistortUsingExternalCoefficients(const Eigen::VectorXd& dist_coeffs,
                                                  Eigen::Vector2d* point) const;

  /// @}

  //////////////////////////////////////////////////////////////
  /// \name Methods to support unit testing.
  /// @{

  /// \brief Create a test distortion object for unit testing.
  static FisheyeDistortion::UniquePtr createTestDistortion() {
    Eigen::VectorXd params(1); params << 1.1;
    return FisheyeDistortion::UniquePtr(new FisheyeDistortion(params));
  }

  /// @}

  ///////////////////////////////////////////////////////////////////////////////
  /// \name Methods to set/get distortion parameters
  /// @{

  /// Static function that checks whether the given intrinsic parameters are valid for this model.
  static bool areParametersValid(const Eigen::VectorXd& parameters);

  /// \brief Check the validity of distortion parameters.
  /// @param[in] dist_coeffs Vector containing the coefficients.
  ///            Parameters will NOT be stored.
  /// @return If the distortion parameters are valid.
  virtual bool distortionParametersValid(const Eigen::VectorXd& dist_coeffs) const;

  /// \brief Returns the number of parameters used in this distortion model.
  inline static constexpr size_t parameterCount() {
      return kNumOfParams;
  }

  /// \brief Returns the number of parameters used in the distortion model.
  ///        NOTE: Use the constexpr function parameterCount if you know the exact distortion type.
  virtual int getParameterSize() const {
    return kNumOfParams;
  }

  /// \brief Print the internal parameters of the distortion in a human-readable form
  /// Print to the ostream that is passed in. The text is extra
  /// text used by the calling function to distinguish cameras.
  virtual void printParameters(std::ostream& out, const std::string& text) const;

  /// @}

  //////////////////////////////////////////////////////////////
  /// \name Valid parameter range definition.
  /// @{

  /// Get the min valid w. W is valid in range [kMinValidW, kMaxValidW].
  static double getMinValidW() {return kMinValidW; }
  /// Get the max valid w. W is valid in range [kMinValidW, kMaxValidW].
  static double getMaxValidW() { return kMaxValidW; }
 private:
  static constexpr double kMaxValidAngle = (89.0 * M_PI / 180.0);
  static constexpr double kMinValidW = 0.5;
  static constexpr double kMaxValidW = 1.5;

  /// @}
};

} // namespace aslam

#include "distortion-fisheye-inl.h"

#endif /* ASLAM_FISHEYE_DISTORTION_H_ */
