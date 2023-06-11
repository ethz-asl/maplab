#include <eigen-checks/gtest.h>
#include <gtest/gtest.h>
#include <opencv2/core/core.hpp>

#include <aslam/cameras/ncamera.h>
#include <aslam/cameras/random-camera-generator.h>
#include <aslam/common/entrypoint.h>
#include <aslam/common/opencv-predicates.h>
#include <aslam/common/unique-id.h>
#include <aslam/frames/visual-frame.h>
#include <aslam/frames/visual-nframe.h>

TEST(NFrame, MinTimestamp) {
  aslam::NCamera::Ptr ncamera = aslam::createSurroundViewTestNCamera();
  CHECK(ncamera);
  aslam::VisualFrame::Ptr frame_0(new aslam::VisualFrame);
  frame_0->setCameraGeometry(ncamera->getCameraShared(0));
  frame_0->setTimestampNanoseconds(123);
  aslam::VisualFrame::Ptr frame_1(new aslam::VisualFrame);
  frame_1->setCameraGeometry(ncamera->getCameraShared(1));
  frame_1->setTimestampNanoseconds(341566);
  aslam::VisualFrame::Ptr frame_2(new aslam::VisualFrame);
  frame_2->setCameraGeometry(ncamera->getCameraShared(2));
  frame_2->setTimestampNanoseconds(98);
  aslam::VisualFrame::Ptr frame_3(new aslam::VisualFrame);
  frame_3->setCameraGeometry(ncamera->getCameraShared(3));
  frame_3->setTimestampNanoseconds(5);
  aslam::NFramesId nframe_id;
  generateId(&nframe_id);
  aslam::VisualNFrame nframe(nframe_id, 4);
  nframe.setNCameras(ncamera);
  nframe.setFrame(0, frame_0);
  nframe.setFrame(1, frame_1);
  nframe.setFrame(2, frame_2);
  nframe.setFrame(3, frame_3);
  int64_t min_timestamp = nframe.getMinTimestampNanoseconds();
  ASSERT_EQ(min_timestamp, 5);
}

TEST(NFrame, CopyConstructor) {
  aslam::NCamera::Ptr ncamera = aslam::createTestNCamera(2);
  aslam::VisualFrame::Ptr frame_0(new aslam::VisualFrame);
  frame_0->setCameraGeometry(ncamera->getCameraShared(0));
  frame_0->setTimestampNanoseconds(123);
  Eigen::Matrix2Xd keypoints = Eigen::Matrix2Xd::Random(2, 10);
  frame_0->setKeypointMeasurements(keypoints);
  aslam::VisualFrame::Ptr frame_1(new aslam::VisualFrame);
  frame_1->setCameraGeometry(ncamera->getCameraShared(1));
  frame_1->setTimestampNanoseconds(341566);
  cv::Mat image = cv::Mat(3, 2, CV_8UC1);
  cv::randu(image, cv::Scalar::all(0), cv::Scalar::all(255));
  frame_1->setRawImage(image);
  aslam::NFramesId nframe_id;
  generateId(&nframe_id);
  aslam::VisualNFrame nframe(nframe_id, 2);
  nframe.setNCameras(ncamera);
  nframe.setFrame(0, frame_0);
  nframe.setFrame(1, frame_1);

  aslam::VisualNFrame nframe_cloned(nframe);
  EXPECT_EQ(nframe.getNumCameras(), nframe_cloned.getNumCameras());
  EXPECT_EQ(nframe.getNCameraShared().get(), nframe_cloned.getNCameraShared().get());
  EIGEN_MATRIX_EQUAL(nframe.getFrame(0).getKeypointMeasurements(),
                     nframe_cloned.getFrame(0).getKeypointMeasurements());
  EXPECT_NEAR_OPENCV(nframe.getFrame(1).getRawImage(), nframe_cloned.getFrame(1).getRawImage(), 0);
  EXPECT_TRUE(nframe == nframe_cloned);
}

ASLAM_UNITTEST_ENTRYPOINT
