#ifndef ASLAM_CAMERAS_CAMERA_3D_LIDAR_H_
#define ASLAM_CAMERAS_CAMERA_3D_LIDAR_H_

#include <aslam/cameras/camera.h>
#include <aslam/cameras/distortion.h>
#include <aslam/common/crtp-clone.h>
#include <aslam/common/macros.h>
#include <aslam/common/types.h>

namespace aslam {

// Forward declarations.
class MappedUndistorter;
class NCamera;

/// \class Camera3DLidar
/// \brief An implementation of a projection model for a 360 degree 3D lidar.
class Camera3DLidar : public aslam::Cloneable<Camera, Camera3DLidar> {
  friend class NCamera;

  enum { kNumOfParams = 4 };

 public:
  ASLAM_POINTER_TYPEDEFS(Camera3DLidar);

  enum { CLASS_SERIALIZATION_VERSION = 1 };
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  enum Parameters {
    kHorizontalResolutionRad = 0,
    kVerticalResolutionRad = 1,
    kVerticalCenterRad = 2,
    kHorizontalCenterRad = 3
  };

  //////////////////////////////////////////////////////////////
  /// \name Constructors/destructors and operators
  /// @{

 public:
  /// \brief Empty constructor for serialization interface.
  Camera3DLidar();

  /// Copy constructor for clone operation.
  Camera3DLidar(const Camera3DLidar& other) = default;
  void operator=(const Camera3DLidar&) = delete;

 public:
  /// \brief Construct a Camera3DLidar
  /// @param[in] intrinsics  (Horizontal-, vertical- resolution in radians,
  /// frequency in Hz).
  /// @param[in] image_width  Image width in pixels.
  /// @param[in] image_height Image height in pixels.
  Camera3DLidar(
      const Eigen::VectorXd& intrinsics, const uint32_t image_width,
      const uint32_t image_height);

  virtual ~Camera3DLidar(){};

  /// \brief Convenience function to print the state using streams.
  friend std::ostream& operator<<(
      std::ostream& out, const Camera3DLidar& camera);

  /// @}

  //////////////////////////////////////////////////////////////
  /// \name Methods to project and back-project euclidean points
  /// @{

  /// \brief Compute the 3d bearing vector in euclidean coordinates given a
  /// keypoint in
  ///        image coordinates. Uses the projection (& distortion) models.
  ///        The result might be in normalized image plane for some camera
  ///        implementations but not for the general case.
  /// @param[in]  keypoint     Keypoint in image coordinates.
  /// @param[out] out_point_3d Bearing vector in euclidean coordinates
  virtual bool backProject3(
      const Eigen::Ref<const Eigen::Vector2d>& keypoint,
      Eigen::Vector3d* out_point_3d) const;

  /// \brief Checks the success of a projection operation and returns the result
  /// in a
  ///        ProjectionResult object.
  /// @param[in] keypoint Keypoint in image coordinates.
  /// @param[in] point_3d Projected point in euclidean.
  /// @return The ProjectionResult object contains details about the success of
  /// the projection.
  template <typename DerivedKeyPoint, typename DerivedPoint3d>
  inline const ProjectionResult evaluateProjectionResult(
      const Eigen::MatrixBase<DerivedKeyPoint>& keypoint,
      const Eigen::MatrixBase<DerivedPoint3d>& point_3d) const;

  /// @}

  //////////////////////////////////////////////////////////////
  /// \name Functional methods to project and back-project points
  /// @{

  // Get the overloaded non-virtual project3Functional(..) from base into scope.
  using Camera::project3Functional;

  /// \brief Template version of project3Functional.
  template <
      typename ScalarType, typename DistortionType, typename MIntrinsics,
      typename MDistortion>
  const ProjectionResult project3Functional(
      const Eigen::Matrix<ScalarType, 3, 1>& point_3d,
      const Eigen::MatrixBase<MIntrinsics>& intrinsics_external,
      const Eigen::MatrixBase<MDistortion>& distortion_coefficients_external,
      Eigen::Matrix<ScalarType, 2, 1>* out_keypoint) const;

  /// \brief This function projects a point into the image using the intrinsic
  /// parameters
  ///        that are passed in as arguments. If any of the Jacobians are
  ///        nonnull, they should be filled in with the Jacobian with respect to
  ///        small changes in the argument.
  /// @param[in]  point_3d                The point in euclidean coordinates.
  /// @param[in]  intrinsics_external     External intrinsic parameter vector.
  ///                                     NOTE: If nullptr, use internal
  ///                                     distortion parameters.
  /// @param[in]  distortion_coefficients_external External distortion parameter
  /// vector.
  ///                                     Parameter is ignored is no distortion
  ///                                     is active. NOTE: If nullptr, use
  ///                                     internal distortion parameters.
  ///                                     NOTE: THIS IS NOT USABLE FOR THIS
  ///                                     CAMERA!
  /// @param[out] out_keypoint            The keypoint in image coordinates.
  ///                                     NOTE: THIS IS NOT USABLE FOR THIS
  ///                                     CAMERA!
  /// @param[out] out_jacobian_point      The Jacobian wrt. to changes in the
  /// euclidean point.
  ///                                       nullptr: calculation is skipped.
  ///                                     NOTE: THIS IS NOT USABLE FOR THIS
  ///                                     CAMERA!
  /// @param[out] out_jacobian_intrinsics The Jacobian wrt. to changes in the
  /// intrinsics.
  ///                                       nullptr: calculation is skipped.
  ///                                     NOTE: THIS IS NOT USABLE FOR THIS
  ///                                     CAMERA!
  /// @param[out] out_jacobian_distortion The Jacobian wrt. to changes in the
  /// distortion parameters.
  ///                                       nullptr: calculation is skipped.
  ///                                     NOTE: THIS IS NOT USABLE FOR THIS
  ///                                     CAMERA!
  /// @return Contains information about the success of the projection. Check
  ///         \ref ProjectionResult for more information.
  virtual const ProjectionResult project3Functional(
      const Eigen::Ref<const Eigen::Vector3d>& point_3d,
      const Eigen::VectorXd* intrinsics_external,
      const Eigen::VectorXd* distortion_coefficients_external,
      Eigen::Vector2d* out_keypoint,
      Eigen::Matrix<double, 2, 3>* out_jacobian_point,
      Eigen::Matrix<double, 2, Eigen::Dynamic>* out_jacobian_intrinsics,
      Eigen::Matrix<double, 2, Eigen::Dynamic>* out_jacobian_distortion) const;

  /// @}

  //////////////////////////////////////////////////////////////
  /// \name Methods to support unit testing.
  /// @{

  /// \brief Creates a random valid keypoint..
  virtual Eigen::Vector2d createRandomKeypoint() const;

  /// \brief Creates a random visible point. Negative depth means random between
  ///        0 and 100 meters.
  virtual Eigen::Vector3d createRandomVisiblePoint(double depth) const;

  /// \brief Get a set of border rays
  void getBorderRays(Eigen::MatrixXd& rays) const;

  /// @}

 public:
  //////////////////////////////////////////////////////////////
  /// \name Methods to access intrinsics.
  /// @{

  /// \brief The vertical resolution of the lidar in radians
  double verticalResolution() const {
    return intrinsics_[Parameters::kVerticalResolutionRad];
  };
  /// \brief The horizontal resolution of the lidar in radians.
  double horizontalResolution() const {
    return intrinsics_[Parameters::kHorizontalResolutionRad];
  };
  /// \brief The horizontal center of the image.
  double horizontalCenter() const {
    return intrinsics_[Parameters::kHorizontalCenterRad];
  };
  /// \brief The vertical center of the image.
  double verticalCenter() const {
    return intrinsics_[Parameters::kVerticalCenterRad];
  };

  /// \brief Returns the number of intrinsic parameters used in this camera
  /// model.
  inline static constexpr int parameterCount() {
    return kNumOfParams;
  }

  /// \brief Returns the number of intrinsic parameters used in this camera
  /// model.
  inline virtual int getParameterSize() const {
    return kNumOfParams;
  }

  /// Static function that checks whether the given intrinsic parameters are
  /// valid for this model.
  static bool areParametersValid(const Eigen::VectorXd& parameters);

  /// Function to check whether the given intrinsic parameters are valid for
  /// this model.
  virtual bool intrinsicsValid(const Eigen::VectorXd& intrinsics) const;

  /// Print the internal parameters of the camera in a human-readable form
  /// Print to the ostream that is passed in. The text is extra
  /// text used by the calling function to distinguish cameras
  virtual void printParameters(
      std::ostream& out, const std::string& text) const;

  /// @}

  /// \brief Create a test camera object for unit testing.
  template <typename DistortionType = NullDistortion>
  static Camera3DLidar::Ptr createTestCamera() {
    return Camera3DLidar::Ptr(
        std::move(createTestCameraUnique<DistortionType>()));
  }

  /// \brief Create a test camera object for unit testing.
  // Simulates OS-1 64-beam lidar.
  template <typename DistortionType = NullDistortion>
  static Camera3DLidar::UniquePtr createTestCameraUnique() {
    Eigen::VectorXd intrinsics(4, 1);
    intrinsics[Parameters::kHorizontalResolutionRad] = 2. * M_PI / 1024.;
    intrinsics[Parameters::kVerticalResolutionRad] =
        0.009203703;  // 0.527333333 deg
    intrinsics[Parameters::kVerticalCenterRad] = 0.289916642;  // 16.611 deg
    intrinsics[Parameters::kHorizontalCenterRad] = 0;

    Camera3DLidar::UniquePtr camera = aligned_unique<Camera3DLidar>(
        intrinsics, 1024u /*width*/, 64u /*height*/);
    CameraId id;
    generateId(&id);
    camera->setId(id);
    return std::move(camera);
  }

  /// \brief The amount of time elapsed between the first row of the image and
  /// the keypoint. For a global shutter camera, this can return Duration(0).
  /// @param[in] keypoint Keypoint to which the delay should be calculated.
  /// @return Time elapsed between the first row of the image and the
  ///         keypoint in nanoseconds.
  int64_t rollingShutterDelayNanoSeconds(
      const Eigen::Vector2d& keypoint) const {
    // Don't check validity. This allows points to wander in and out
    // of the frame during optimization
    return static_cast<int64_t>(keypoint(0)) * line_delay_nanoseconds_;
  }

  /// \brief The amount of time elapsed between the first row of the image and
  /// the last row of the image. For a global shutter camera, this can return
  /// 0.
  int64_t rollingShutterDelayNanoSeconds() const {
    return this->imageWidth() * line_delay_nanoseconds_;
  }

  /// \brief returns the number of lines, this is either the number of columns
  /// or rows or 1 in case of a global shutter camera.
  uint32_t getNumberOfLines() const {
    if (line_delay_nanoseconds_ > 0) {
      return this->imageWidth();
    }
    return 1u;
  }

  /// \brief Returns which orientation the rolling shutter effect is occuring.
  LineDelayMode getLineDelayMode() const {
    return LineDelayMode::kColumns;
  }

 private:
  /// \brief Minimal depth for a valid projection.
  static const double kSquaredMinimumDepth;

  bool isValidImpl() const override;
  void setRandomImpl() override;
  bool isEqualImpl(const Sensor& other, const bool verbose) const override;
};

}  // namespace aslam

#include "aslam/cameras/camera-3d-lidar-inl.h"

#endif  // ASLAM_CAMERAS_CAMERA_3D_LIDAR_H_
