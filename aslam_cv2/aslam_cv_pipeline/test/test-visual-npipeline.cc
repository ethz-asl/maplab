#include <gtest/gtest.h>
#include <opencv2/core/core.hpp>

#include <aslam/cameras/camera.h>
#include <aslam/cameras/camera-pinhole.h>
#include <aslam/cameras/distortion-radtan.h>
#include <aslam/cameras/ncamera.h>
#include <aslam/common/entrypoint.h>
#include <aslam/frames/visual-nframe.h>
#include <aslam/pipeline/visual-pipeline-null.h>
#include <aslam/pipeline/visual-pipeline.h>
#include <aslam/pipeline/visual-npipeline.h>

using namespace aslam;

class VisualNPipelineTest : public ::testing::Test {
 protected:
  typedef aslam::RadTanDistortion DistortionType;
  typedef aslam::PinholeCamera CameraType;

  void constructNCamera(unsigned num_cameras,
                        unsigned num_threads,
                        int64_t timestamp_tolerance_ns) {
    NCameraId id;
    generateId(&id);
    Aligned<std::vector, kindr::minimal::QuatTransformation> T_C_B;
    std::vector<Camera::Ptr> cameras;
    std::vector<VisualPipeline::Ptr> pipelines;
    for(unsigned i = 0; i < num_cameras; ++i) {
      kindr::minimal::QuatTransformation T;
      T_C_B.push_back(T);

      CameraType::Ptr camera = CameraType::createTestCamera<DistortionType>();
      cameras.push_back(camera);
      pipelines.push_back(std::shared_ptr<VisualPipeline>(new NullVisualPipeline(camera, false)));
    }
    camera_rig_.reset(new NCamera(id, T_C_B, cameras, "Test Camera System"));

    pipeline_.reset(new VisualNPipeline(num_threads, pipelines,
                                        camera_rig_, camera_rig_,
                                        timestamp_tolerance_ns));

  }

  cv::Mat getImageFromCamera(unsigned camera_index) {
    CHECK_NOTNULL(camera_rig_.get());
    CHECK_LT(camera_index, camera_rig_->getNumCameras());
    return cv::Mat(this->camera_rig_->getCamera(camera_index).imageHeight(),
                   this->camera_rig_->getCamera(camera_index).imageWidth(),
                   CV_8UC1, uint8_t(camera_index));
  }

  NCamera::Ptr camera_rig_;
  VisualNPipeline::Ptr pipeline_;
};

TEST_F(VisualNPipelineTest, buildNFramesOutOfOrder) {
  this->constructNCamera(2, 4, 100);

  // Build n frames out of order.
  pipeline_->processImage(0, getImageFromCamera(0), 0);
  pipeline_->waitForAllWorkToComplete();
  ASSERT_EQ(0u, pipeline_->getNumFramesComplete());

  pipeline_->processImage(0, getImageFromCamera(0), 1000);
  pipeline_->waitForAllWorkToComplete();
  ASSERT_EQ(0u, pipeline_->getNumFramesComplete());

  pipeline_->processImage(1, getImageFromCamera(0), 1);
  pipeline_->waitForAllWorkToComplete();
  ASSERT_EQ(1u, pipeline_->getNumFramesComplete());

  pipeline_->processImage(1, getImageFromCamera(0), 1001);
  pipeline_->waitForAllWorkToComplete();
  ASSERT_EQ(2u, pipeline_->getNumFramesComplete());

  std::shared_ptr<VisualNFrame> nframes = pipeline_->getNext();

  ASSERT_EQ(1u, pipeline_->getNumFramesComplete());
  ASSERT_TRUE(nframes.get() != NULL);
  ASSERT_EQ(0, nframes->getFrame(0).getTimestampNanoseconds());
  ASSERT_EQ(1, nframes->getFrame(1).getTimestampNanoseconds());

  nframes = pipeline_->getNext();
  ASSERT_EQ(0u, pipeline_->getNumFramesComplete());
  ASSERT_TRUE(nframes.get() != NULL);
  ASSERT_EQ(1000, nframes->getFrame(0).getTimestampNanoseconds());
  ASSERT_EQ(1001, nframes->getFrame(1).getTimestampNanoseconds());
}

TEST_F(VisualNPipelineTest, testBuildAndClear) {
  this->constructNCamera(2, 4, 100);

  // Check that the clearing of older completed frames works.
  pipeline_->processImage(0, getImageFromCamera(0), 0);
  pipeline_->waitForAllWorkToComplete();
  ASSERT_EQ(0u, pipeline_->getNumFramesComplete());

  pipeline_->processImage(0, getImageFromCamera(0), 1000);
  pipeline_->waitForAllWorkToComplete();
  ASSERT_EQ(0u, pipeline_->getNumFramesComplete());

  pipeline_->processImage(1, getImageFromCamera(0), 1);
  pipeline_->waitForAllWorkToComplete();
  ASSERT_EQ(1u, pipeline_->getNumFramesComplete());

  pipeline_->processImage(1, getImageFromCamera(0), 1001);
  pipeline_->waitForAllWorkToComplete();
  ASSERT_EQ(2u, pipeline_->getNumFramesComplete());

  std::shared_ptr<VisualNFrame> nframes = pipeline_->getLatestAndClear();

  ASSERT_EQ(0u, pipeline_->getNumFramesComplete());
  ASSERT_TRUE(nframes.get() != NULL);
  ASSERT_EQ(1000, nframes->getFrame(0).getTimestampNanoseconds());
  ASSERT_EQ(1001, nframes->getFrame(1).getTimestampNanoseconds());
}

TEST_F(VisualNPipelineTest, testTimestampDiff) {
  this->constructNCamera(2, 4, 100);

  // Check that the timestamp tolerance is respected.
  pipeline_->processImage(0, getImageFromCamera(0), 0);
  pipeline_->waitForAllWorkToComplete();
  ASSERT_EQ(0u, pipeline_->getNumFramesComplete());

  pipeline_->processImage(1, getImageFromCamera(1), 100);
  pipeline_->waitForAllWorkToComplete();
  ASSERT_EQ(1u, pipeline_->getNumFramesComplete());
  std::shared_ptr<VisualNFrame> nframes = pipeline_->getLatestAndClear();
  ASSERT_TRUE(nframes.get() != NULL);
  ASSERT_EQ(0, nframes->getFrame(0).getTimestampNanoseconds());
  ASSERT_EQ(100, nframes->getFrame(1).getTimestampNanoseconds());

  // Build n frames out of order.
  pipeline_->processImage(0, getImageFromCamera(0), 0);
  pipeline_->waitForAllWorkToComplete();
  ASSERT_EQ(0u, pipeline_->getNumFramesComplete());

  pipeline_->processImage(1, getImageFromCamera(1), 101);
  pipeline_->waitForAllWorkToComplete();
  ASSERT_EQ(2u, pipeline_->getNumFramesProcessing());  // 0, 101
  ASSERT_EQ(0u, pipeline_->getNumFramesComplete());    // -

  // Get the latest. This should be null as no frames are complete.
  nframes = pipeline_->getNext();
  ASSERT_TRUE(nframes.get() == NULL);
  // And there should still be two processing
  ASSERT_EQ(2u, pipeline_->getNumFramesProcessing());

  // Add an even later frame
  pipeline_->processImage(0, getImageFromCamera(0), 401);
  pipeline_->waitForAllWorkToComplete();
  ASSERT_EQ(3u, pipeline_->getNumFramesProcessing());  // 0, 101, 401
  ASSERT_EQ(0u, pipeline_->getNumFramesComplete());    // -

  // Finish the middle frame, but it should still not be moved to the output queue.
  // As there are nframes still processing with lower timestamps and the output is
  // chronologically ordered. This leaves us still with 3 processing and 0 completed.
  pipeline_->processImage(0, getImageFromCamera(0), 101);
  pipeline_->waitForAllWorkToComplete();
  ASSERT_EQ(3u, pipeline_->getNumFramesProcessing());  // 0, 101, 401
  ASSERT_EQ(0u, pipeline_->getNumFramesComplete());    // -

  // Get the latest. This should be null as no frames are complete.
  nframes = pipeline_->getNext();
  ASSERT_TRUE(nframes.get() == NULL);

  // Finish the first queued frame. This should leave us with two frames (0, 101) in the
  // output queue.
  pipeline_->processImage(1, getImageFromCamera(1), 0);
  pipeline_->waitForAllWorkToComplete();
  ASSERT_EQ(1u, pipeline_->getNumFramesProcessing());  // 401
  ASSERT_EQ(2u, pipeline_->getNumFramesComplete());    // 0, 101

  // Check output queue content and ordering.
  nframes = pipeline_->getNext();
  ASSERT_TRUE(nframes.get() != NULL);
  ASSERT_EQ(0, nframes->getFrame(0).getTimestampNanoseconds());
  ASSERT_EQ(0, nframes->getFrame(1).getTimestampNanoseconds());

  nframes = pipeline_->getNext();
  ASSERT_TRUE(nframes.get() != NULL);
  ASSERT_EQ(101, nframes->getFrame(0).getTimestampNanoseconds());
  ASSERT_EQ(101, nframes->getFrame(1).getTimestampNanoseconds());

  // The queue should now be empty.
  ASSERT_EQ(1u, pipeline_->getNumFramesProcessing());  // 401
  ASSERT_EQ(0u, pipeline_->getNumFramesComplete());    // -

  // Now we finish the last frame and check it.
  pipeline_->processImage(1, getImageFromCamera(1), 401);
  pipeline_->waitForAllWorkToComplete();
  ASSERT_EQ(0u, pipeline_->getNumFramesProcessing());  // -
  ASSERT_EQ(1u, pipeline_->getNumFramesComplete());    // 401

  nframes = pipeline_->getNext();
  ASSERT_TRUE(nframes.get() != NULL);
  ASSERT_EQ(401, nframes->getFrame(0).getTimestampNanoseconds());
  ASSERT_EQ(401, nframes->getFrame(1).getTimestampNanoseconds());

  ASSERT_EQ(0u, pipeline_->getNumFramesProcessing());
  ASSERT_EQ(0u, pipeline_->getNumFramesComplete());

  nframes = pipeline_->getNext();
  ASSERT_TRUE(nframes.get() == NULL);
}

ASLAM_UNITTEST_ENTRYPOINT
