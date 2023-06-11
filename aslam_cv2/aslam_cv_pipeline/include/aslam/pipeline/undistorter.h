#ifndef ASLAM_UNDISTORTER_H_
#define ASLAM_UNDISTORTER_H_

#include <aslam/cameras/camera.h>
#include <aslam/common/macros.h>

// Forward declarations.
namespace cv { class Mat; };

namespace aslam {

/// \class Undistorter
/// \brief A base class for image undistortion and resizing.
class Undistorter {
public:
  ASLAM_POINTER_TYPEDEFS(Undistorter);
  ASLAM_DISALLOW_EVIL_CONSTRUCTORS(Undistorter);

protected:
  Undistorter() {};

public:
  /// \brief Construct an undistorter pipeline from the input and output cameras
  ///
  /// \param[in] input_camera  The intrinsics associated with the input image.
  /// \param[in] output_camera The intrinsics associated with the output image.
  Undistorter(Camera::Ptr input_camera, Camera::Ptr output_camera);

  virtual ~Undistorter() {};

  /// \brief Produce an undistorted image from an input image.
  virtual void processImage(const cv::Mat& input_image, cv::Mat* output_image) const = 0;

  /// \brief Get the input camera that corresponds to the image
  ///        passed in to processImage().
  ///
  /// Because this processor may do things like image undistortion or
  /// rectification, the input and output camera may not be the same.
  const Camera& getInputCamera() const { return *CHECK_NOTNULL(input_camera_.get()); };

  /// \brief Get the input camera that corresponds to the image
  ///        passed in to processImage().
  ///
  /// Because this processor may do things like image undistortion or
  /// rectification, the input and output camera may not be the same.
  Camera::Ptr getInputCameraShared() { return input_camera_; };

  /// \brief Get the input camera that corresponds to the image
  ///        passed in to processImage().
  ///
  /// Because this processor may do things like image undistortion or
  /// rectification, the input and output camera may not be the same.
  Camera::ConstPtr getInputCameraShared() const { return input_camera_; };

  /// \brief Get the output camera that corresponds to the VisualFrame
  ///        data that comes out.
  ///
  /// Because this pipeline may do things like image undistortion or
  /// rectification, the input and output camera may not be the same.
  const Camera& getOutputCamera() const { return *CHECK_NOTNULL(output_camera_.get()); };

  /// \brief Get the output camera that corresponds to the VisualFrame
  ///        data that comes out.
  ///
  /// Because this pipeline may do things like image undistortion or
  /// rectification, the input and output camera may not be the same.
  Camera::Ptr getOutputCameraShared() { return output_camera_; };

  /// \brief Get the output camera that corresponds to the VisualFrame
  ///        data that comes out.
  ///
  /// Because this pipeline may do things like image undistortion or
  /// rectification, the input and output camera may not be the same.
  Camera::ConstPtr getOutputCameraShared() const { return output_camera_; };

protected:
  /// \brief The intrinsics of the raw image.
  Camera::Ptr input_camera_;
  /// \brief The intrinsics of the raw image.
  Camera::Ptr output_camera_;
};

} // namespace aslam

#endif // ASLAM_UNDISTORTER_H_
