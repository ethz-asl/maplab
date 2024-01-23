#ifndef ASLAM_CAMERAS_CAMERA_H_
#define ASLAM_CAMERAS_CAMERA_H_

#include <cstdint>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <glog/logging.h>

#include <aslam/cameras/distortion-null.h>
#include <aslam/cameras/distortion.h>
#include <aslam/common/macros.h>
#include <aslam/common/sensor.h>
#include <aslam/common/types.h>
#include <aslam/common/unique-id.h>

// TODO(slynen) Enable commented out PropertyTree support
// namespace sm {
// class PropertyTree;
//}

namespace aslam {

// Forward declarations
class MappedUndistorter;

/// \struct ProjectionResult
/// \brief This struct is returned by the camera projection methods and holds
/// the result state
///        of the projection operation.
struct ProjectionResult {
  /// Possible projection state.
  enum class Status {
    /// Keypoint is visible and projection was successful.
    KEYPOINT_VISIBLE,
    /// Keypoint is NOT visible but projection was successful.
    KEYPOINT_OUTSIDE_IMAGE_BOX,
    /// The projected point lies behind the camera plane.
    POINT_BEHIND_CAMERA,
    /// The projection was unsuccessful.
    PROJECTION_INVALID,
    /// Default value after construction.
    UNINITIALIZED
  };
  // Make the enum values accessible from the outside without the additional
  // indirection.
  static Status KEYPOINT_VISIBLE;
  static Status KEYPOINT_OUTSIDE_IMAGE_BOX;
  static Status POINT_BEHIND_CAMERA;
  static Status PROJECTION_INVALID;
  static Status UNINITIALIZED;

  constexpr ProjectionResult() : status_(Status::UNINITIALIZED){};
  constexpr ProjectionResult(Status status) : status_(status){};

  /// \brief ProjectionResult can be typecasted to bool and is true if the
  /// projected keypoint
  ///        is visible. Simplifies the check for a successful projection.
  ///        Example usage:
  /// @code
  ///          aslam::ProjectionResult ret =
  ///          camera_->project3(Eigen::Vector3d(0, 0, -10), &keypoint); if(ret)
  ///          std::cout << "Projection was successful!\n";
  /// @endcode
  explicit operator bool() const {
    return isKeypointVisible();
  };

  /// \brief Compare objects.
  bool operator==(const ProjectionResult& other) const {
    return status_ == other.status_;
  };

  /// \brief Compare projection status.
  bool operator==(const ProjectionResult::Status& other) const {
    return status_ == other;
  };

  /// \brief Convenience function to print the state using streams.
  friend std::ostream& operator<<(
      std::ostream& out, const ProjectionResult& state);

  /// \brief Check whether the projection was successful and the point is
  /// visible in the image.
  bool isKeypointVisible() const {
    return (status_ == Status::KEYPOINT_VISIBLE);
  };

  /// \brief Returns the exact state of the projection operation.
  ///        Example usage:
  /// @code
  ///          aslam::ProjectionResult ret =
  ///          camera_->project3(Eigen::Vector3d(0, 0, -1), &keypoint);
  ///          if(ret.getDetailedStatus() ==
  ///          aslam::ProjectionResult::Status::KEYPOINT_OUTSIDE_IMAGE_BOX)
  ///            std::cout << "Point behind camera! Lets do something...\n";
  /// @endcode
  Status getDetailedStatus() const {
    return status_;
  };

 private:
  /// Stores the projection state.
  Status status_;
};

/// \class Camera
/// \brief The base camera class provides methods to project/backproject
/// euclidean and
///        homogeneous points. The actual projection is implemented in the
///        derived classes for euclidean coordinates only; homogeneous
///        coordinates are support by a conversion. The intrinsic parameters are
///        documented in the specialized camera classes.
class Camera : public Sensor {
 public:
  ASLAM_POINTER_TYPEDEFS(Camera);
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  enum { CLASS_SERIALIZATION_VERSION = 1 };

  enum class Type {
    kPinhole = 0,
    kUnifiedProjection = 1,
    kLidar3D = 2,
  };

  //////////////////////////////////////////////////////////////
  /// \name Constructors/destructors and operators
  /// @{

  // TODO(slynen) Enable commented out PropertyTree support
  // explicit Camera(const sm::PropertyTree& property_tree);
 protected:
  Camera() = delete;

  /// \brief Camera base constructor with distortion.
  /// @param[in] intrinsics   Vector containing the intrinsic parameters.
  /// @param[in] distortion   unique_ptr to the distortion model
  /// @param[in] image_width  Image width in pixels.
  /// @param[in] image_height Image height in pixels.
  /// @param[in] camera_type  CameraType enum value with information which
  ///            camera  model is used by the derived class.
  Camera(
      const Eigen::VectorXd& intrinsics,
      aslam::Distortion::UniquePtr& distortion, const uint32_t image_width,
      const uint32_t image_height, Type camera_type);

  /// \brief Camera base constructor without distortion.
  /// @param[in] intrinsics   Vector containing the intrinsic parameters.
  /// @param[in] image_width  Image width in pixels.
  /// @param[in] image_height Image height in pixels.
  /// @param[in] camera_type  CameraType enum value with information which
  ///            camera model is used by the derived class.
  Camera(
      const Eigen::VectorXd& intrinsics, const uint32_t image_width,
      const uint32_t image_height, Type camera_type);

  Sensor::Ptr cloneAsSensor() const override {
    return std::dynamic_pointer_cast<Sensor>(std::shared_ptr<Camera>(clone()));
  }

 public:
  virtual ~Camera() = default;

  /// \brief Convenience function to print the state using streams.
  std::ostream& operator<<(std::ostream& out) {
    this->printParameters(out, std::string(""));
    return out;
  };

  /// \brief Clones the camera instance and returns a pointer to the copy.
  virtual aslam::Camera* clone() const = 0;

  /// Get sensor type as an integer or as a string
  uint8_t getSensorType() const override {
    return SensorType::kCamera;
  }

  std::string getSensorTypeString() const override {
    return static_cast<std::string>(kCameraIdentifier);
  }

 protected:
  /// Copy constructor for clone operation.
  Camera(const Camera& other)
      : Sensor(other),
        line_delay_nanoseconds_(other.line_delay_nanoseconds_),
        image_width_(other.image_width_),
        image_height_(other.image_height_),
        mask_(other.mask_.clone()),
        is_compressed_(other.is_compressed_),
        intrinsics_(other.intrinsics_),
        camera_type_(other.camera_type_),
        distortion_(nullptr) {
    CHECK(other.distortion_);
    distortion_.reset(other.distortion_->clone());
  };

  void operator=(const Camera&) = delete;

  /// \brief Compare only the parameters of Camera to the ones of another Camera
  bool isEqualCameraImpl(const Camera& other, const bool verbose = false) const;

  /// @}

  //////////////////////////////////////////////////////////////
  /// \name Information about the camera
  /// @{
 public:
  /// \brief The width of the image in pixels.
  uint32_t imageWidth() const {
    return image_width_;
  }

  /// \brief The height of the image in pixels.
  uint32_t imageHeight() const {
    return image_height_;
  }

  /// \brief Set the width of the image in pixels.
  void setImageWidth(uint32_t image_width) {
    image_width_ = image_width;
  }

  /// \brief Set the height of the image in pixels.
  void setImageHeight(uint32_t image_height) {
    image_height_ = image_height;
  }

  /// \brief Print the internal parameters of the camera in a human-readable
  /// form Print to the ostream that is passed in. The text is extra text used
  /// by the calling function to distinguish cameras.
  virtual void printParameters(
      std::ostream& out, const std::string& text) const;

  /// \brief The number of intrinsic parameters.
  virtual int getParameterSize() const = 0;

  /// \brief Returns type of the camera model.
  /// @return CameraType value representing the camera model used by the derived
  /// class.
  inline Type getType() const {
    return camera_type_;
  }

  /// @}

  //////////////////////////////////////////////////////////////
  /// \name Methods to project and back-project euclidean points
  /// @{

  /// \brief Projects a euclidean point to a 2d image measurement. Applies the
  ///        projection (& distortion) models to the point.
  /// @param[in]  point_3d     The point in euclidean coordinates.
  /// @param[out] out_keypoint The keypoint in image coordinates.
  /// @return Contains information about the success of the projection. Check
  ///         \ref ProjectionResult for more information.
  const ProjectionResult project3(
      const Eigen::Ref<const Eigen::Vector3d>& point_3d,
      Eigen::Vector2d* out_keypoint) const;

  /// \brief Projects a euclidean point to a 2d image measurement. Applies the
  ///        projection (& distortion) models to the po int.
  /// @param[in]  point_3d     The point in euclidean coordinates.
  /// @param[out] out_keypoint The keypoint in image coordinates.
  /// @param[out] out_jacobian The Jacobian wrt. to changes in the euclidean
  /// point.
  /// @return Contains information about the success of the projection. Check
  ///         \ref ProjectionResult for more information.
  const ProjectionResult project3(
      const Eigen::Ref<const Eigen::Vector3d>& point_3d,
      Eigen::Vector2d* out_keypoint,
      Eigen::Matrix<double, 2, 3>* out_jacobian) const;

  /// \brief Projects a matrix of euclidean points to 2d image measurements.
  /// Applies the
  ///        projection (& distortion) models to the points.
  ///
  /// This vanilla version just repeatedly calls backProject3. Camera
  /// implementers are encouraged to override for efficiency.
  /// @param[in]  point_3d      The point in euclidean coordinates.
  /// @param[out] out_keypoints The keypoint in image coordinates.
  /// @param[out] out_results   Contains information about the success of the
  ///                           projections. Check \ref ProjectionResult for
  ///                           more information.
  virtual void project3Vectorized(
      const Eigen::Ref<const Eigen::Matrix3Xd>& points_3d,
      Eigen::Matrix2Xd* out_keypoints,
      std::vector<ProjectionResult>* out_results) const;

  /// \brief Compute the 3d bearing vector in euclidean coordinates given a
  /// keypoint in
  ///        image coordinates. Uses the projection (& distortion) models.
  ///        The result might be in normalized image plane for some camera
  ///        implementations but not for the general case.
  /// @param[in]  keypoint     Keypoint in image coordinates.
  /// @param[out] out_point_3d Bearing vector in euclidean coordinates
  /// @return Was the projection successful?
  virtual bool backProject3(
      const Eigen::Ref<const Eigen::Vector2d>& keypoint,
      Eigen::Vector3d* out_point_3d) const = 0;

  /// \brief Compute the 3d bearing vectors in euclidean coordinates given a
  /// list of
  ///        keypoints in image coordinates. Uses the projection (& distortion)
  ///        models.
  ///
  /// This vanilla version just repeatedly calls backProject3. Camera
  /// implementers are encouraged to override for efficiency.
  /// TODO(schneith): implement efficient backProject3Vectorized
  /// @param[in]  keypoints     Keypoints in image coordinates.
  /// @param[out] out_point_3ds Bearing vectors in euclidean coordinates (with
  /// z=1 -> non-normalized).
  /// @param[out] out_success   Were the projections successful?
  virtual void backProject3Vectorized(
      const Eigen::Ref<const Eigen::Matrix2Xd>& keypoints,
      Eigen::Matrix3Xd* out_points_3d,
      std::vector<unsigned char>* out_success) const;
  /// @}

  //////////////////////////////////////////////////////////////
  /// \name Methods to project and back-project homogeneous points
  /// @{

  /// \brief Projects a homogeneous point to a 2d image measurement. Applies the
  ///        projection (& distortion) models to the point.
  /// @param[in]  point_4d     The point in homogeneous coordinates.
  /// @param[out] out_keypoint The keypoint in image coordinates.
  /// @return Contains information about the success of the projection. Check
  ///         \ref ProjectionResult for more information.
  const ProjectionResult project4(
      const Eigen::Ref<const Eigen::Vector4d>& point_4d,
      Eigen::Vector2d* out_keypoint) const;

  /// \brief Projects a euclidean point to a 2d image measurement. Applies the
  ///        projection (& distortion) models to the point.
  /// @param[in]  point_4d     The point in homogeneous coordinates.
  /// @param[out] out_keypoint The keypoint in image coordinates.
  /// @param[out] out_jacobian The Jacobian wrt. to changes in the homogeneous
  /// point.
  /// @return Contains information about the success of the projection. Check
  /// \ref
  ///         ProjectionResult for more information.
  const ProjectionResult project4(
      const Eigen::Ref<const Eigen::Vector4d>& point_4d,
      Eigen::Vector2d* out_keypoint,
      Eigen::Matrix<double, 2, 4>* out_jacobian) const;

  /// \brief Compute the 3d bearing vector in homogeneous coordinates given a
  /// keypoint in
  ///        image coordinates. Uses the projection (& distortion) models.
  /// @param[in]  keypoint     Keypoint in image coordinates.
  /// @param[out] out_point_3d Bearing vector in homogeneous coordinates.
  /// @return Was the projection successful?
  bool backProject4(
      const Eigen::Ref<const Eigen::Vector2d>& keypoint,
      Eigen::Vector4d* out_point_4d) const;

  /// @}

  //////////////////////////////////////////////////////////////
  /// \name Functional methods to project and back-project points
  /// @{

  /// \brief This function projects a point into the image using the intrinsic
  /// parameters
  ///        that are passed in as arguments. If any of the Jacobians are
  ///        nonnull, they should be filled in with the Jacobian with respect to
  ///        small changes in the argument.
  /// @param[in]  point_3d                The point in euclidean coordinates.
  /// @param[in]  intrinsics_external     External intrinsic parameter vector.
  ///                                     NOTE: If nullptr, use internal
  ///                                     intrinsic parameters.
  /// @param[in]  distortion_coefficients_external External distortion parameter
  /// vector.
  ///                                     Parameter is ignored is no distortion
  ///                                     is active. NOTE: If nullptr, use
  ///                                     internal distortion parameters.
  /// @param[out] out_keypoint            The keypoint in image coordinates.
  /// @return Contains information about the success of the projection. Check
  /// \ref
  ///         ProjectionResult for more information.
  const ProjectionResult project3Functional(
      const Eigen::Ref<const Eigen::Vector3d>& point_3d,
      const Eigen::VectorXd* intrinsics_external,
      const Eigen::VectorXd* distortion_coefficients_external,
      Eigen::Vector2d* out_keypoint) const;

  /// \brief This function projects a point into the image using the intrinsic
  /// parameters
  ///        that are passed in as arguments. If any of the Jacobians are
  ///        nonnull, they should be filled in with the Jacobian with respect to
  ///        small changes in the argument.
  /// @param[in]  point_3d                The point in euclidean coordinates.
  /// @param[in]  intrinsics_external     External intrinsic parameter vector.
  ///                                     NOTE: If nullptr, use internal
  ///                                     intrinsic parameters.
  /// @param[in]  distortion_coefficients_external External distortion parameter
  /// vector.
  ///                                     Parameter is ignored is no distortion
  ///                                     is active. NOTE: If nullptr, use
  ///                                     internal distortion parameters.
  /// @param[out] out_keypoint            The keypoint in image coordinates.
  /// @param[out] out_jacobian_point      The Jacobian wrt. to changes in the
  /// euclidean point.
  ///                                       nullptr: calculation is skipped.
  /// @param[out] out_jacobian_intrinsics The Jacobian wrt. to changes in the
  /// intrinsics.
  ///                                       nullptr: calculation is skipped.
  /// @param[out] out_jacobian_distortion The Jacobian wrt. to changes in the
  /// distortion parameters.
  ///                                       nullptr: calculation is skipped.
  /// @return Contains information about the success of the projection. Check
  /// \ref
  ///         ProjectionResult for more information.
  virtual const ProjectionResult project3Functional(
      const Eigen::Ref<const Eigen::Vector3d>& point_3d,
      const Eigen::VectorXd* intrinsics_external,
      const Eigen::VectorXd* distortion_coefficients_external,
      Eigen::Vector2d* out_keypoint,
      Eigen::Matrix<double, 2, 3>* out_jacobian_point,
      Eigen::Matrix<double, 2, Eigen::Dynamic>* out_jacobian_intrinsics,
      Eigen::Matrix<double, 2, Eigen::Dynamic>* out_jacobian_distortion)
      const = 0;

  /// @}

 public:
  //////////////////////////////////////////////////////////////
  /// \name Methods to support rolling shutter cameras
  /// @{

  /// \brief  Return the temporal offset of a rolling shutter camera.
  ///         Global shutter cameras return zero.
  /// @return Line delay in nano seconds.
  uint64_t getLineDelayNanoSeconds() const {
    return line_delay_nanoseconds_;
  }

  /// \brief Set the temporal offset of a rolling shutter camera.
  /// @param[in] line_delay_nano_seconds Line delay in nano seconds.
  void setLineDelayNanoSeconds(uint64_t line_delay_nano_seconds) {
    line_delay_nanoseconds_ = line_delay_nano_seconds;
  }

  /// \brief The amount of time elapsed between the first row of the image and
  /// the keypoint. For a global shutter camera, this can return Duration(0).
  /// @param[in] keypoint Keypoint to which the delay should be calculated.
  /// @return Time elapsed between the first row of the image and the
  ///         keypoint in nanoseconds.
  virtual int64_t rollingShutterDelayNanoSeconds(
      const Eigen::Vector2d& keypoint) const {
    // Don't check validity. This allows points to wander in and out
    // of the frame during optimization
    return static_cast<int64_t>(keypoint(1)) * line_delay_nanoseconds_;
  }

  /// \brief The amount of time elapsed between the first row of the image and
  /// the
  ///        last row of the image. For a global shutter camera, this can return
  ///        0.
  virtual int64_t maxRollingShutterDelayNanoSeconds() const {
    return this->imageHeight() * line_delay_nanoseconds_;
  }

  /// \brief returns the number of lines, this is either the number of columns
  /// or rows or 1 in case of a global shutter camera.
  virtual uint32_t getNumberOfLines() const {
    if (line_delay_nanoseconds_ > 0) {
      return this->imageHeight();
    }
    return 1u;
  }

  enum class LineDelayMode : uint8_t { kColumns = 0, kRows = 1 };

  /// \brief Returns which orientation the rolling shutter effect is occuring.
  virtual LineDelayMode getLineDelayMode() const {
    return LineDelayMode::kRows;
  }
  /// @}

  //////////////////////////////////////////////////////////////
  /// \name Methods to support compressed images.
  /// @{

  /// \brief Holds whether this camera receives compressed images.
  /// @return Returns true for compressed images, false otherwise
  bool hasCompressedImages() const {
    return is_compressed_;
  }

  /// \brief Set the whether this camera has compressed images.
  /// @param[in] is_compressed Compressed images.
  void setCompressedImages(const bool is_compressed) {
    is_compressed_ = is_compressed;
  }

  /// @}

  //////////////////////////////////////////////////////////////
  /// \name Methods to test validity and visibility
  /// @{

  /// \brief Can the projection function be run on this point? This doesn't test
  /// if
  ///        the projected point is visible, only if the projection function can
  ///        be run without numerical errors or singularities.
  bool isProjectable3(const Eigen::Ref<const Eigen::Vector3d>& point) const;

  /// \brief  Can the projection function be run on this point? This doesn't
  /// test
  ///         if the projected point is visible, only if the projection function
  ///         can be run without numerical errors or singularities.
  bool isProjectable4(const Eigen::Ref<const Eigen::Vector4d>& point) const;

  /// \brief  Return if a given keypoint is inside the imaging box of the
  /// camera.
  template <typename DerivedKeyPoint>
  bool isKeypointVisible(
      const Eigen::MatrixBase<DerivedKeyPoint>& keypoint) const;

  /// \brief  Return if a given keypoint is within the specified margin to the
  ///         boundary of the imaging box of the camera.
  template <typename DerivedKeyPoint>
  bool isKeypointVisibleWithMargin(
      const Eigen::MatrixBase<DerivedKeyPoint>& keypoint,
      typename DerivedKeyPoint::Scalar margin) const;

  /// @}

  //////////////////////////////////////////////////////////////
  /// \name Methods to support unit testing.
  /// @{

  /// Creates a random valid keypoint.
  virtual Eigen::Vector2d createRandomKeypoint() const = 0;

  /// Creates a random visible point. Negative depth means random between 0 and
  /// 100 meters.
  virtual Eigen::Vector3d createRandomVisiblePoint(double depth) const = 0;

  /// @}

  //////////////////////////////////////////////////////////////
  /// \name Methods to interface the underlying distortion model.
  /// @{

  /// Returns a pointer to the underlying distortion object.
  aslam::Distortion* getDistortionMutable() {
    return CHECK_NOTNULL(distortion_.get());
  };

  /// Returns the underlying distortion object.
  const aslam::Distortion& getDistortion() const {
    return *CHECK_NOTNULL(distortion_.get());
  };

  /// Set the distortion model.
  void setDistortion(aslam::Distortion::UniquePtr& distortion) {
    distortion_ = std::move(distortion);
  };

  /// Is a distortion model set for this camera.
  bool hasDistortion() const {
    CHECK(distortion_);
    return distortion_->getType() != aslam::Distortion::Type::kNoDistortion;
  }

  /// Remove the distortion model from this camera.
  void removeDistortion() {
    distortion_.reset(new NullDistortion);
  };
  /// @}

  //////////////////////////////////////////////////////////////
  /// \name Methods to access the intrinsic parameters.
  /// @{

  /// Get the intrinsic parameters (const).
  inline const Eigen::VectorXd& getParameters() const {
    return intrinsics_;
  };

  /// Get the intrinsic parameters.
  inline double* getParametersMutable() {
    return &intrinsics_.coeffRef(0, 0);
  };

  /// Set the intrinsic parameters. Parameters are documented in the specialized
  /// camera classes.
  void setParameters(const Eigen::VectorXd& params) {
    CHECK_EQ(getParameterSize(), params.size());
    intrinsics_ = params;
  }

  /// Function to check whether the given intrinsic parameters are valid for
  /// this model.
  virtual bool intrinsicsValid(const Eigen::VectorXd& intrinsics) const = 0;

  /// @}

  //////////////////////////////////////////////////////////////
  /// \name Methods to access the mask.
  /// @{

  /// Set the mask. Masks must be the same size as the image and they follow the
  /// same convention as OpenCV: 0 == masked, nonzero == valid.
  void setMask(const cv::Mat& mask);

  /// Get the mask.
  const cv::Mat& getMask() const;

  /// Clear the mask.
  void clearMask();

  /// Does the camera have a mask?
  bool hasMask() const;

  /// Check if the keypoint is masked.
  inline bool isMasked(
      const Eigen::Ref<const Eigen::Vector2d>& keypoint) const {
    return keypoint[0] < 0.0 ||
           keypoint[0] >= static_cast<double>(image_width_) ||
           keypoint[1] < 0.0 ||
           keypoint[1] >= static_cast<double>(image_height_) ||
           (!mask_.empty() && mask_.at<uint8_t>(
                                  static_cast<int>(keypoint[1]),
                                  static_cast<int>(keypoint[0])) == 0);
  }

  /// @}

  /// \name Factory Methods
  /// @{

  /// \brief A factory function to create a derived class camera
  ///
  /// This function takes a vectors of intrinsics and distortion parameters
  /// and produces a camera.
  /// \param[in] intrinsics  A vector of projection intrinsic parameters.
  /// \param[in] imageWidth  The width of the image associated with this camera.
  /// \param[in] imageHeight The height of the image associated with this
  /// camera. \param[in] distortionParameters The parameters of the distortion
  /// object. \returns A new camera based on the template types.
  template <typename DerivedCamera, typename DerivedDistortion>
  static typename DerivedCamera::Ptr construct(
      const Eigen::VectorXd& intrinsics, uint32_t imageWidth,
      uint32_t imageHeight, const Eigen::VectorXd& distortionParameters);

  /// @}

 private:
  bool isValidImpl() const = 0;
  void setRandomImpl() = 0;
  bool isEqualImpl(const Sensor& other, const bool verbose) const = 0;

  bool loadFromYamlNodeImpl(const YAML::Node&) override;
  void saveToYamlNodeImpl(YAML::Node*) const override;

 protected:
  /// The delay per scanline for a rolling shutter camera in nanoseconds.
  uint64_t line_delay_nanoseconds_;
  /// The width of the image.
  uint32_t image_width_;
  /// The height of the image.
  uint32_t image_height_;
  /// The image mask.
  cv::Mat_<uint8_t> mask_;
  /// Has compressed images.
  bool is_compressed_;

  /// Parameter vector for the intrinsic parameters of the model.
  Eigen::VectorXd intrinsics_;

  /// \brief Enum field to store the type of camera model.
  Type camera_type_;

  /// \brief The distortion for this camera.
  aslam::Distortion::UniquePtr distortion_;
};
}  // namespace aslam
#include "camera-inl.h"
#endif  // ASLAM_CAMERAS_CAMERA_H_
