#include <aslam/pipeline/visual-pipeline.h>

#include <aslam/cameras/camera.h>
#include <aslam/frames/visual-frame.h>

#include <opencv2/core/core.hpp>

namespace aslam {

VisualPipeline::VisualPipeline(const Camera::ConstPtr& input_camera,
                               const Camera::ConstPtr& output_camera, bool copy_images)
: input_camera_(input_camera), output_camera_(output_camera),
  copy_images_(copy_images) {
  CHECK(input_camera);
  CHECK(output_camera);
}

std::shared_ptr<VisualFrame> VisualPipeline::processImage(const cv::Mat& raw_image,
                                                          int64_t timestamp) const {
  CHECK_EQ(input_camera_->imageWidth(), static_cast<size_t>(raw_image.cols));
  CHECK_EQ(input_camera_->imageHeight(), static_cast<size_t>(raw_image.rows));

  // \TODO(PTF) Eventually we can put timestamp correction policies in here.
  std::shared_ptr<VisualFrame> frame(new VisualFrame);
  frame->setTimestampNanoseconds(timestamp);
  frame->setRawCameraGeometry(input_camera_);
  frame->setCameraGeometry(output_camera_);
  FrameId id;
  generateId(&id);
  frame->setId(id);
  if(copy_images_) {
    frame->setRawImage(raw_image.clone());
  } else {
    frame->setRawImage(raw_image);
  }

  /// Send the image to the derived class for processing
  processFrameImpl(raw_image, frame.get());

  return frame;
}

}  // namespace aslam
