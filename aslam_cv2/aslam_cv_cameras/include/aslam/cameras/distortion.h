#ifndef ASLAM_CAMERAS_DISTORTION_H_
#define ASLAM_CAMERAS_DISTORTION_H_

#include <aslam/common/macros.h>
#include <Eigen/Dense>
#include <gflags/gflags.h>

DECLARE_double(acv_inv_distortion_tolerance);

namespace aslam {

/// \class Distortion
/// \brief This class represents a standard implementation of the distortion block. The function
///        "distort" applies this nonlinear transformation. The function "undistort" applies the
///        inverse transformation.
class Distortion {
 public:
  ASLAM_POINTER_TYPEDEFS(Distortion);
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  enum { CLASS_SERIALIZATION_VERSION = 1 };

  enum class Type {
    kNoDistortion = 0,
    kEquidistant = 1,
    kFisheye = 2,
    kRadTan = 3
  };

  //////////////////////////////////////////////////////////////
  /// \name Constructors/destructors and operators
  /// @{

 protected:
  Distortion() = delete;

  /// \brief Distortion base constructor.
  /// @param[in] dist_coeffs     Vector containing the distortion parameters.
  /// @param[in] distortion_type DistortionType enum value with information which distortion
  ///                            model is used by the derived class.
  Distortion(const Eigen::VectorXd& dist_coeffs,
             Type distortion_type);

 public:
  virtual ~Distortion() { };

  /// \brief Checks for same distortion type and same parameters.
  /// @return Same distortion?
  virtual bool operator==(const Distortion& rhs) const;

  /// \brief Convenience function to print the state using streams.
  friend std::ostream& operator<<(std::ostream& out, const Distortion& distortion);

 protected:
  /// Copy constructor for clone operation.
  Distortion(const Distortion&) = default;
  void operator=(const Distortion&) = delete;

 public:
  /// \brief Clones the camera instance and returns a pointer to the copy.
  virtual aslam::Distortion* clone() const = 0;

  /// @}

  //////////////////////////////////////////////////////////////
  /// \name Distort methods: applies the distortion model to a point.
  /// @{

  /// \brief Apply distortion to a point in the normalized image plane
  /// @param[in,out] point The point in the normalized image plane. After the function,
  ///                      this point is distorted.
  void distort(Eigen::Vector2d* point) const;

  /// \brief Apply distortion to a point in the normalized image plane
  /// @param[in]  point     The point in the normalized image plane.
  /// @param[out] out_point The distorted point.
  void distort(const Eigen::Vector2d& point, Eigen::Vector2d* out_point) const;

  /// \brief Apply distortion to a point in the normalized image plane
  /// @param[in,out] point        The point in the normalized image plane. After the function,
  ///                             this point is distorted.
  /// @param[out]    out_jacobian The Jacobian of the distortion function with respect to small
  ///                             changes in the input point.
  void distort(Eigen::Vector2d* point,
               Eigen::Matrix2d* out_jacobian) const;

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
                                                Eigen::Matrix2d* out_jacobian) const = 0;

  /// \brief Apply distortion to the point and provide the Jacobian of the distortion with respect
  ///        to small changes in the distortion parameters.
  /// @param[in]  dist_coeffs  Vector containing the coefficients for the distortion model.
  ///                          NOTE: If nullptr, use internal distortion parameters.
  /// @param[in]  point        The point in the normalized image plane.
  /// @param[out] out_jacobian The Jacobian of the distortion with respect to small changes in
  ///                          the distortion parameters.
  virtual void distortParameterJacobian(const Eigen::VectorXd* dist_coeffs,
                                 const Eigen::Vector2d& point,
                                 Eigen::Matrix<double, 2, Eigen::Dynamic>* out_jacobian) const = 0;

  /// @}

  //////////////////////////////////////////////////////////////
  /// \name Undistort methods: Removes the modeled distortion effects from a point.
  /// @{

  /// \brief Apply undistortion to recover a point in the normalized image plane.
  /// @param[in,out] point The distorted point. After the function, this point is in
  ///                      the normalized image plane.
  void undistort(Eigen::Vector2d* point) const;

  /// \brief Apply undistortion to recover a point in the normalized image plane.
  /// @param[in]    point     External distortion coefficients to use.
  /// @param[out]   out_point The undistorted point in normalized image plane.
  void undistort(const Eigen::Vector2d& point, Eigen::Vector2d* out_point) const;


  /// \brief Apply undistortion to recover a point in the normalized image plane using provided
  ///        distortion coefficients. External distortion coefficients can be specified using this
  ///        function. Ignores the internally stored parameters.
  /// @param[in]     dist_coeffs  Vector containing the coefficients for the distortion model.
  /// @param[in,out] point        The distorted point. After the function, this point is in the
  ///                             normalized image plane.
  virtual void undistortUsingExternalCoefficients(const Eigen::VectorXd& dist_coeffs,
                                                  Eigen::Vector2d* point) const = 0;

  /// @}

  //////////////////////////////////////////////////////////////
  /// \name Methods to set/get distortion parameters.
  /// @{

  /// \brief Set the coefficients for the distortion model.
  /// @param[in] dist_coeffs Vector containing the coefficients.
  void setParameters(const Eigen::VectorXd& dist_coeffs);

  /// \brief Get the distortion model coefficients.
  /// @return Vector containing the coefficients.
  inline const Eigen::VectorXd& getParameters() const { return distortion_coefficients_; };

  /// \brief Get the pointer to the distortion model coefficients.
  /// @return Pointer to the first coefficient.
  inline double* getParametersMutable() { return &distortion_coefficients_.coeffRef(0, 0); };

  /// \brief Returns the number of parameters used in the distortion model.
  ///        NOTE: Use the constexpr function parameterCount if you know the exact distortion type.
  virtual int getParameterSize() const = 0;

  /// \brief Check the validity of distortion parameters.
  /// @param[in] dist_coeffs Vector containing the coefficients. Parameters will NOT be stored.
  /// @return If the distortion parameters are valid.
  virtual bool distortionParametersValid(const Eigen::VectorXd& dist_coeffs) const = 0;
  
  /// \brief Print the internal parameters of the distortion in a human-readable form
  /// Print to the ostream that is passed in. The text is extra
  /// text used by the calling function to distinguish cameras.
  virtual void printParameters(std::ostream& out, const std::string& text) const = 0;

  /// \brief Returns type of the distortion model.
  /// @return DistortionType value representing the distortion model used by the derived class.
  inline Type getType() const { return distortion_type_; }

  /// @}

 protected:
  /// \brief Parameter vector for the distortion model.
  Eigen::VectorXd distortion_coefficients_;

  /// \brief Enum field to store the type of distortion model.
  Type distortion_type_;
};
}  // namespace aslam
#include "distortion-inl.h"
#endif  // ASLAM_CAMERAS_DISTORTION_H_
