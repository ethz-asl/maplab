#ifndef NULL_VISUAL_PIPELINE_H
#define NULL_VISUAL_PIPELINE_H

#include "visual-pipeline.h"

namespace aslam {

/// \class NullVisualPipeline
/// \brief A visual pipeline that does not transform the image.
class NullVisualPipeline : public VisualPipeline {
public:
  ASLAM_POINTER_TYPEDEFS(NullVisualPipeline);
  ASLAM_DISALLOW_EVIL_CONSTRUCTORS(NullVisualPipeline);

  /// \param[in] camera The camera that produces the images.
  /// \param[in] copyImages If true, images passed in are cloned before storing
  ///                       in the frame.
  NullVisualPipeline(const Camera::ConstPtr& camera, bool copyImages);

  virtual ~NullVisualPipeline() {};

protected:
  /// \brief Process the frame and fill the results into the frame variable
  ///
  /// The top level function will already fill in the timestamps and the output camera.
  /// \param[in]     image The image data.
  /// \param[in/out] frame The visual frame. This will be constructed before calling.
  virtual void processFrameImpl(const cv::Mat& /* image */,
                                VisualFrame* /* frame */) const { }

};

}  // namespace aslam

#endif // NULL_VISUAL_PIPELINE_H
