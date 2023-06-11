#ifndef VISUAL_PROCESSOR_H
#define VISUAL_PROCESSOR_H

#include <memory>

#include <opencv2/core/core.hpp>

#include <aslam/cameras/camera.h>
#include <aslam/common/macros.h>
#include <aslam/pipeline/undistorter.h>
#include <aslam/frames/visual-frame.h>

namespace aslam {
/// \class VisualPipeline
/// \brief An interface for processors that turn images into VisualFrames
///
/// This is the abstract interface for visual processors that turn raw images
/// into VisualFrame data. The underlying pipeline may include undistortion
/// or rectification, image contrast enhancement, feature detection and
/// descriptor computation, or other operations.
///
/// The class has two Camera calibration structs that represent the
/// extrinsic calibration of the camera.
/// The "input" calibration (getInputCamera()) represents the calibration of
/// raw camera, before any image processing, resizing, or undistortion
/// has taken place. The "output" calibration (getOutputCamera())
/// represents the calibration parameters of the images and keypoints that get
/// set in the VisualFrame struct. These are the camera parameters after
/// image processing, resizing, undistortion, etc.
class VisualPipeline {
public:
  ASLAM_POINTER_TYPEDEFS(VisualPipeline);
  ASLAM_DISALLOW_EVIL_CONSTRUCTORS(VisualPipeline);

protected:
  VisualPipeline() : copy_images_(false) {};

public:
  /// \brief Construct a visual pipeline from the input and output cameras
  ///
  /// \param[in] input_camera  The intrinsics associated with the raw image.
  /// \param[in] output_camera The intrinsics associated with the keypoints.
  /// \param[in] copy_images    Should we copy the image before storing it in the frame?
  VisualPipeline(const Camera::ConstPtr& input_camera, const Camera::ConstPtr& output_camera,
                 bool copy_images);

  /// \brief Construct a visual pipeline from the input and output cameras
  ///
  /// \param[in] preprocessing Preprocessing to apply to the image before sending to the pipeline.
  /// \param[in] copy_images    Should we copy the image before storing it in the frame?
  VisualPipeline(std::unique_ptr<Undistorter>& preprocessing, bool copy_images);

  virtual ~VisualPipeline() {};

  /// \brief Add an image to the visual processor.
  ///
  /// This function is called by a user when an image is received.
  /// The processor then processes the images and constructs a VisualFrame.
  /// This method constructs a basic frame and passes it on to processFrame().
  ///
  /// \param[in] image          The image data.
  /// \param[in] timestamp      The time in integer nanoseconds.
  /// \returns                  The visual frame built from the image data.
  VisualFrame::Ptr processImage(const cv::Mat& image, int64_t timestamp) const;

  /// \brief Get the input camera that corresponds to the image
  ///        passed in to processImage().
  ///
  /// Because this processor may do things like image undistortion or
  /// rectification, the input and output camera may not be the same.
  const Camera& getInputCamera() const { CHECK(input_camera_); return *input_camera_; }

  /// \brief Get the output camera that corresponds to the VisualFrame
  ///        data that comes out.
  ///
  /// Because this pipeline may do things like image undistortion or
  /// rectification, the input and output camera may not be the same.
  const Camera& getOutputCamera() const { CHECK(output_camera_); return *output_camera_; }

  /// \brief Get a shared pointer to the input camera that corresponds to the image
  ///        passed in to processImage().
  ///
  /// Because this processor may do things like image undistortion or
  /// rectification, the input and output camera may not be the same.
  Camera::ConstPtr getInputCameraShared() const { return input_camera_; }

  /// \brief Get a shared pointer to the output camera that corresponds to the VisualFrame
  ///        data that comes out.
  ///
  /// Because this pipeline may do things like image undistortion or
  /// rectification, the input and output camera may not be the same.
  Camera::ConstPtr getOutputCameraShared() const { return output_camera_; }

protected:
  /// \brief Process the frame and fill the results into the frame variable.
  ///
  /// This function can be used to chain together pipelines that do different things.
  /// The top level function will already fill in the timestamps and the output camera.
  /// \param[in]     image The image data.
  /// \param[in/out] frame The visual frame. This will be constructed before calling.
  virtual void processFrameImpl(const cv::Mat& image,
                                VisualFrame* frame) const = 0;

  /// \brief Preprocessing for the image. Can be null.
  const std::unique_ptr<Undistorter> preprocessing_;
  /// \brief The intrinsics of the raw image.
  std::shared_ptr<const Camera> input_camera_;
  /// \brief The intrinsics of the raw image.
  std::shared_ptr<const Camera> output_camera_;
  /// \brief Should we copy the image before storing it in the frame?
  bool copy_images_;
};
}  // namespace aslam

#endif // VISUAL_PROCESSOR_H
