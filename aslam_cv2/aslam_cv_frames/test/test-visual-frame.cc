#include <eigen-checks/gtest.h>
#include <gtest/gtest.h>

#include <aslam/cameras/camera-pinhole.h>
#include <aslam/cameras/camera-unified-projection.h>
#include <aslam/common/channel-declaration.h>
#include <aslam/common/entrypoint.h>
#include <aslam/common/opencv-predicates.h>
#include <aslam/cameras/camera.h>
#include <aslam/frames/visual-frame.h>

TEST(Frame, SetGetCamera) {
  aslam::Camera::Ptr camera;
  aslam::VisualFrame frame;
  ASSERT_FALSE(static_cast<bool>(frame.getCameraGeometry()));
  frame.setCameraGeometry(camera);
  EXPECT_EQ(camera, frame.getCameraGeometry());
}

TEST(Frame, DeathGetElementFromOnUnsetData) {
  aslam::VisualFrame frame;
  EXPECT_DEATH(frame.getDescriptor(0), "^");
  EXPECT_DEATH(frame.getKeypointMeasurement(0), "^");
  EXPECT_DEATH(frame.getKeypointMeasurementUncertainty(0), "^");
  EXPECT_DEATH(frame.getKeypointScale(0), "^");
  EXPECT_DEATH(frame.getKeypointOrientation(0), "^");
}

TEST(Frame, DeathOnGetUnsetData) {
  aslam::VisualFrame frame;
  EXPECT_DEATH(frame.getDescriptors(), "^");
  EXPECT_DEATH(frame.getKeypointMeasurements(), "^");
  EXPECT_DEATH(frame.getKeypointMeasurementUncertainties(), "^");
  EXPECT_DEATH(frame.getKeypointScales(), "^");
  EXPECT_DEATH(frame.getKeypointOrientations(), "^");
  EXPECT_DEATH(frame.getRawImage(), "^");
}

TEST(Frame, DeathOnGetMutableUnsetData) {
  aslam::VisualFrame frame;
  EXPECT_DEATH(frame.getDescriptorsMutable(), "^");
  EXPECT_DEATH(frame.getKeypointMeasurementsMutable(), "^");
  EXPECT_DEATH(frame.getKeypointMeasurementUncertaintiesMutable(), "^");
  EXPECT_DEATH(frame.getKeypointScalesMutable(), "^");
  EXPECT_DEATH(frame.getKeypointOrientationsMutable(), "^");
  EXPECT_DEATH(frame.getRawImageMutable(), "^");
}

TEST(Frame, SetGetDescriptors) {
  aslam::VisualFrame frame;
  aslam::VisualFrame::DescriptorsT data(48, 10);
  data.setRandom();
  frame.setDescriptors(data);
  {
    const aslam::VisualFrame::DescriptorsT& data_2 =
        frame.getDescriptors();
    EXPECT_TRUE(EIGEN_MATRIX_EQUAL(data, data_2));
    EXPECT_EQ(&data_2, frame.getDescriptorsMutable());
    for (int i = 0; i < data.cols(); ++i) {
      const unsigned char* data_ptr = frame.getDescriptor(i);
      EXPECT_EQ(&data_2.coeffRef(0, i), data_ptr);
    }
  }

  // Test extending
  aslam::VisualFrame::DescriptorsT data_ext(64, 12);
  data_ext.setRandom();
  frame.extendDescriptors(data_ext, 1);
  {
    const aslam::VisualFrame::DescriptorsT& data_2 =
        frame.getDescriptors(0);
    const aslam::VisualFrame::DescriptorsT& data_3 =
        frame.getDescriptors(1);
    EXPECT_TRUE(EIGEN_MATRIX_EQUAL(data, data_2));
    EXPECT_TRUE(EIGEN_MATRIX_EQUAL(data_ext, data_3));
    EXPECT_EQ(&data_2, frame.getDescriptorsMutable(0));
    EXPECT_EQ(&data_3, frame.getDescriptorsMutable(1));
    for (int i = 0; i < data.cols(); ++i) {
      const unsigned char* data_ptr = frame.getDescriptor(i);
      EXPECT_EQ(&data_2.coeffRef(0, i), data_ptr);
    }
    for (int i = 0; i < data_ext.cols(); ++i) {
      const size_t index = data.cols() + i;
      const unsigned char* data_ptr = frame.getDescriptor(index);
      EXPECT_EQ(&data_3.coeffRef(0, i), data_ptr);
    }

    EXPECT_EQ(data_2, frame.getDescriptorsOfType(0));
    EXPECT_EQ(data_3, frame.getDescriptorsOfType(1));
  }
}

TEST(Frame, SetGetKeypointMeasurements) {
  aslam::VisualFrame frame;
  Eigen::Matrix2Xd data(2, 10);
  data.setRandom();
  frame.setKeypointMeasurements(data);
  const Eigen::Matrix2Xd& data_2 = frame.getKeypointMeasurements();
  EXPECT_TRUE(EIGEN_MATRIX_NEAR(data, data_2, 1e-6));
  EXPECT_EQ(&data_2, frame.getKeypointMeasurementsMutable());
  for (int i = 0; i < data.cols(); ++i) {
    const Eigen::Vector2d& ref = frame.getKeypointMeasurement(i);
    const Eigen::Vector2d& should = data.block<2, 1>(0, i);
    EXPECT_TRUE(EIGEN_MATRIX_NEAR(should, ref, 1e-6));
  }

  // Test extending
  Eigen::Matrix2Xd data_ext(2, 20);
  data_ext.setRandom();
  frame.extendKeypointMeasurements(data_ext);
  const Eigen::Matrix2Xd& data_3 = frame.getKeypointMeasurements();
  EXPECT_TRUE(EIGEN_MATRIX_NEAR(data, data_3.block(0, 0, 2, 10), 1e-6));
  EXPECT_TRUE(EIGEN_MATRIX_NEAR(data_ext, data_3.block(0, 10, 2, 20), 1e-6));

  // Some descriptors are necessary in the frame since the keypoint type
  // is defined by the descriptor type
  aslam::VisualFrame::DescriptorsT desc_1(8, 10);
  aslam::VisualFrame::DescriptorsT desc_2(8, 20);
  desc_1.setRandom();
  desc_2.setRandom();
  frame.setDescriptors(desc_1, 0);
  frame.extendDescriptors(desc_2, 1);

  const Eigen::Block<const Eigen::Matrix2Xd> data_subset_1 =
      frame.getKeypointMeasurementsOfType(0);
  const Eigen::Block<const Eigen::Matrix2Xd> data_subset_2 =
      frame.getKeypointMeasurementsOfType(1);

  EXPECT_TRUE(EIGEN_MATRIX_NEAR(data, data_subset_1, 1e-6));
  EXPECT_TRUE(EIGEN_MATRIX_NEAR(data_ext, data_subset_2, 1e-6));

  for (int i = 0; i < 10; i++) {
    const Eigen::Vector2d& ref = frame.getKeypointMeasurementOfType(i, 0);
    const Eigen::Vector2d& should = data.block<2, 1>(0, i);
    EXPECT_TRUE(EIGEN_MATRIX_NEAR(should, ref, 1e-6));
  }

  for (int i = 0; i < 20; i++) {
    const Eigen::Vector2d& ref = frame.getKeypointMeasurementOfType(i, 1);
    const Eigen::Vector2d& should = data_ext.block<2, 1>(0, i);
    EXPECT_TRUE(EIGEN_MATRIX_NEAR(should, ref, 1e-6));
  }

  // Check memory addresses again and that no copy operations happened
  CHECK_EQ(&(data_3.coeff(0,0)), &(data_subset_1.coeff(0,0)));
  CHECK_EQ(&(data_3.coeff(0,10)), &(data_subset_2.coeff(0,0)));
}

TEST(Frame, SetGetKeypointMeasurementUncertainties) {
  aslam::VisualFrame frame;
  Eigen::VectorXd data(10);
  data.setRandom();
  frame.setKeypointMeasurementUncertainties(data);
  const Eigen::VectorXd& data_2 = frame.getKeypointMeasurementUncertainties();
  EXPECT_TRUE(EIGEN_MATRIX_NEAR(data, data_2, 1e-6));
  EXPECT_EQ(&data_2, frame.getKeypointMeasurementUncertaintiesMutable());
  for (int i = 0; i < data.cols(); ++i) {
    double ref = frame.getKeypointMeasurementUncertainty(i);
    EXPECT_NEAR(data(i), ref, 1e-6);
  }

  // Test extending as well as the default padding
  Eigen::Matrix2Xd keypoints(2, 40);
  keypoints.setRandom();
  frame.setKeypointMeasurements(keypoints);

  Eigen::VectorXd data_ext(20);
  data_ext.setRandom();
  frame.extendKeypointMeasurementUncertainties(data_ext, 0.0);
  const Eigen::VectorXd& data_3 = frame.getKeypointMeasurementUncertainties();
  EXPECT_EQ(data_3.size(), keypoints.cols());

  Eigen::VectorXd data_zero(10);
  data_zero.setConstant(0.0);
  EXPECT_TRUE(EIGEN_MATRIX_NEAR(data, data_3.segment(0, 10), 1e-6));
  EXPECT_TRUE(EIGEN_MATRIX_NEAR(data_zero, data_3.segment(10, 10), 1e-6));
  EXPECT_TRUE(EIGEN_MATRIX_NEAR(data_ext, data_3.segment(20, 20), 1e-6));

  // Some descriptors are necessary in the frame since the keypoint type
  // is defined by the descriptor type
  aslam::VisualFrame::DescriptorsT desc_1(8, 10);
  aslam::VisualFrame::DescriptorsT desc_2(8, 10);
  aslam::VisualFrame::DescriptorsT desc_3(8, 20);
  desc_1.setRandom();
  desc_2.setRandom();
  desc_3.setRandom();
  frame.setDescriptors(desc_1, 0);
  frame.extendDescriptors(desc_2, 1);
  frame.extendDescriptors(desc_3, 2);

  const Eigen::VectorBlock<const Eigen::VectorXd> data_subset_1 =
      frame.getKeypointMeasurementUncertaintiesOfType(0);
  const Eigen::VectorBlock<const Eigen::VectorXd> data_subset_2 =
      frame.getKeypointMeasurementUncertaintiesOfType(1);
  const Eigen::VectorBlock<const Eigen::VectorXd> data_subset_3 =
      frame.getKeypointMeasurementUncertaintiesOfType(2);

  EXPECT_TRUE(EIGEN_MATRIX_NEAR(data, data_subset_1, 1e-6));
  EXPECT_TRUE(EIGEN_MATRIX_NEAR(data_zero, data_subset_2, 1e-6));
  EXPECT_TRUE(EIGEN_MATRIX_NEAR(data_ext, data_subset_3, 1e-6));

  for (int i = 0; i < 10; i++) {
    const double ref = frame.getKeypointMeasurementUncertaintyOfType(i, 0);
    EXPECT_EQ(data(i), ref);
  }

  for (int i = 0; i < 10; i++) {
    const double ref = frame.getKeypointMeasurementUncertaintyOfType(i, 1);
    EXPECT_EQ(data_zero(i), ref);
  }

  for (int i = 0; i < 20; i++) {
    const double ref = frame.getKeypointMeasurementUncertaintyOfType(i, 2);
    EXPECT_EQ(data_ext(i), ref);
  }

  // Check memory addresses again and that no copy operations happened
  CHECK_EQ(&(data_3.coeff(0)), &(data_subset_1.coeff(0)));
  CHECK_EQ(&(data_3.coeff(10)), &(data_subset_2.coeff(0)));
  CHECK_EQ(&(data_3.coeff(20)), &(data_subset_3.coeff(0)));
}

TEST(Frame, SetGetKeypointOrientations) {
  aslam::VisualFrame frame;
  Eigen::VectorXd data(10);
  data.setRandom();
  frame.setKeypointOrientations(data);
  const Eigen::VectorXd& data_2 = frame.getKeypointOrientations();
  EXPECT_TRUE(EIGEN_MATRIX_NEAR(data, data_2, 1e-6));
  EXPECT_EQ(&data_2, frame.getKeypointOrientationsMutable());
  for (int i = 0; i < data.cols(); ++i) {
    double ref = frame.getKeypointOrientation(i);
    EXPECT_NEAR(data(i), ref, 1e-6);
  }

  // Test extending as well as the default padding
  Eigen::Matrix2Xd keypoints(2, 40);
  keypoints.setRandom();
  frame.setKeypointMeasurements(keypoints);

  Eigen::VectorXd data_ext(20);
  data_ext.setRandom();
  frame.extendKeypointOrientations(data_ext, 0.0);
  const Eigen::VectorXd& data_3 = frame.getKeypointOrientations();
  EXPECT_EQ(data_3.size(), keypoints.cols());

  Eigen::VectorXd data_zero(10);
  data_zero.setConstant(0.0);
  EXPECT_TRUE(EIGEN_MATRIX_NEAR(data, data_3.segment(0, 10), 1e-6));
  EXPECT_TRUE(EIGEN_MATRIX_NEAR(data_zero, data_3.segment(10, 10), 1e-6));
  EXPECT_TRUE(EIGEN_MATRIX_NEAR(data_ext, data_3.segment(20, 20), 1e-6));

  // Some descriptors are necessary in the frame since the keypoint type
  // is defined by the descriptor type
  aslam::VisualFrame::DescriptorsT desc_1(8, 10);
  aslam::VisualFrame::DescriptorsT desc_2(8, 10);
  aslam::VisualFrame::DescriptorsT desc_3(8, 20);
  desc_1.setRandom();
  desc_2.setRandom();
  desc_3.setRandom();
  frame.setDescriptors(desc_1, 0);
  frame.extendDescriptors(desc_2, 1);
  frame.extendDescriptors(desc_3, 2);

  const Eigen::VectorBlock<const Eigen::VectorXd> data_subset_1 =
      frame.getKeypointOrientationsOfType(0);
  const Eigen::VectorBlock<const Eigen::VectorXd> data_subset_2 =
      frame.getKeypointOrientationsOfType(1);
  const Eigen::VectorBlock<const Eigen::VectorXd> data_subset_3 =
      frame.getKeypointOrientationsOfType(2);

  EXPECT_TRUE(EIGEN_MATRIX_NEAR(data, data_subset_1, 1e-6));
  EXPECT_TRUE(EIGEN_MATRIX_NEAR(data_zero, data_subset_2, 1e-6));
  EXPECT_TRUE(EIGEN_MATRIX_NEAR(data_ext, data_subset_3, 1e-6));

  for (int i = 0; i < 10; i++) {
    const double ref = frame.getKeypointOrientationOfType(i, 0);
    EXPECT_EQ(data(i), ref);
  }

  for (int i = 0; i < 10; i++) {
    const double ref = frame.getKeypointOrientationOfType(i, 1);
    EXPECT_EQ(data_zero(i), ref);
  }

  for (int i = 0; i < 20; i++) {
    const double ref = frame.getKeypointOrientationOfType(i, 2);
    EXPECT_EQ(data_ext(i), ref);
  }

  // Check memory addresses again and that no copy operations happened
  CHECK_EQ(&(data_3.coeff(0)), &(data_subset_1.coeff(0)));
  CHECK_EQ(&(data_3.coeff(10)), &(data_subset_2.coeff(0)));
  CHECK_EQ(&(data_3.coeff(20)), &(data_subset_3.coeff(0)));
}

TEST(Frame, SetGetKeypointScales) {
  aslam::VisualFrame frame;
  Eigen::VectorXd data;
  data.resize(10);
  data.setRandom();
  frame.setKeypointScales(data);
  const Eigen::VectorXd& data_2 = frame.getKeypointScales();
  EXPECT_TRUE(EIGEN_MATRIX_NEAR(data, data_2, 1e-6));
  EXPECT_EQ(&data_2, frame.getKeypointScalesMutable());
  for (int i = 0; i < data.cols(); ++i) {
    double ref = frame.getKeypointScale(i);
    EXPECT_NEAR(data(i), ref, 1e-6);
  }

  // Test extending as well as the default padding
  Eigen::Matrix2Xd keypoints(2, 40);
  keypoints.setRandom();
  frame.setKeypointMeasurements(keypoints);

  Eigen::VectorXd data_ext(20);
  data_ext.setRandom();
  frame.extendKeypointScales(data_ext, 0.0);
  const Eigen::VectorXd& data_3 = frame.getKeypointScales();
  EXPECT_EQ(data_3.size(), keypoints.cols());

  Eigen::VectorXd data_zero(10);
  data_zero.setConstant(0.0);
  EXPECT_TRUE(EIGEN_MATRIX_NEAR(data, data_3.segment(0, 10), 1e-6));
  EXPECT_TRUE(EIGEN_MATRIX_NEAR(data_zero, data_3.segment(10, 10), 1e-6));
  EXPECT_TRUE(EIGEN_MATRIX_NEAR(data_ext, data_3.segment(20, 20), 1e-6));

  // Some descriptors are necessary in the frame since the keypoint type
  // is defined by the descriptor type
  aslam::VisualFrame::DescriptorsT desc_1(8, 10);
  aslam::VisualFrame::DescriptorsT desc_2(8, 10);
  aslam::VisualFrame::DescriptorsT desc_3(8, 20);
  desc_1.setRandom();
  desc_2.setRandom();
  desc_3.setRandom();
  frame.setDescriptors(desc_1, 0);
  frame.extendDescriptors(desc_2, 1);
  frame.extendDescriptors(desc_3, 2);

  const Eigen::VectorBlock<const Eigen::VectorXd> data_subset_1 =
      frame.getKeypointScalesOfType(0);
  const Eigen::VectorBlock<const Eigen::VectorXd> data_subset_2 =
      frame.getKeypointScalesOfType(1);
  const Eigen::VectorBlock<const Eigen::VectorXd> data_subset_3 =
      frame.getKeypointScalesOfType(2);

  EXPECT_TRUE(EIGEN_MATRIX_NEAR(data, data_subset_1, 1e-6));
  EXPECT_TRUE(EIGEN_MATRIX_NEAR(data_zero, data_subset_2, 1e-6));
  EXPECT_TRUE(EIGEN_MATRIX_NEAR(data_ext, data_subset_3, 1e-6));

  for (int i = 0; i < 10; i++) {
    const double ref = frame.getKeypointScaleOfType(i, 0);
    EXPECT_EQ(data(i), ref);
  }

  for (int i = 0; i < 10; i++) {
    const double ref = frame.getKeypointScaleOfType(i, 1);
    EXPECT_EQ(data_zero(i), ref);
  }

  for (int i = 0; i < 20; i++) {
    const double ref = frame.getKeypointScaleOfType(i, 2);
    EXPECT_EQ(data_ext(i), ref);
  }

  // Check memory addresses again and that no copy operations happened
  CHECK_EQ(&(data_3.coeff(0)), &(data_subset_1.coeff(0)));
  CHECK_EQ(&(data_3.coeff(10)), &(data_subset_2.coeff(0)));
  CHECK_EQ(&(data_3.coeff(20)), &(data_subset_3.coeff(0)));
}

TEST(Frame, SetGetKeypointScores) {
  aslam::VisualFrame frame;
  Eigen::VectorXd data;
  data.resize(10);
  data.setRandom();
  frame.setKeypointScores(data);
  const Eigen::VectorXd& data_2 = frame.getKeypointScores();
  EXPECT_TRUE(EIGEN_MATRIX_NEAR(data, data_2, 1e-6));
  EXPECT_EQ(&data_2, frame.getKeypointScoresMutable());
  for (int i = 0; i < data.cols(); ++i) {
    double ref = frame.getKeypointScore(i);
    EXPECT_NEAR(data(i), ref, 1e-6);
  }

  // Test extending as well as the default padding
  Eigen::Matrix2Xd keypoints(2, 40);
  keypoints.setRandom();
  frame.setKeypointMeasurements(keypoints);

  Eigen::VectorXd data_ext(20);
  data_ext.setRandom();
  frame.extendKeypointScores(data_ext, 0.0);
  const Eigen::VectorXd& data_3 = frame.getKeypointScores();
  EXPECT_EQ(data_3.size(), keypoints.cols());

  Eigen::VectorXd data_zero(10);
  data_zero.setConstant(0.0);
  EXPECT_TRUE(EIGEN_MATRIX_NEAR(data, data_3.segment(0, 10), 1e-6));
  EXPECT_TRUE(EIGEN_MATRIX_NEAR(data_zero, data_3.segment(10, 10), 1e-6));
  EXPECT_TRUE(EIGEN_MATRIX_NEAR(data_ext, data_3.segment(20, 20), 1e-6));

  // Some descriptors are necessary in the frame since the keypoint type
  // is defined by the descriptor type
  aslam::VisualFrame::DescriptorsT desc_1(8, 10);
  aslam::VisualFrame::DescriptorsT desc_2(8, 10);
  aslam::VisualFrame::DescriptorsT desc_3(8, 20);
  desc_1.setRandom();
  desc_2.setRandom();
  desc_3.setRandom();
  frame.setDescriptors(desc_1, 0);
  frame.extendDescriptors(desc_2, 1);
  frame.extendDescriptors(desc_3, 2);

  const Eigen::VectorBlock<const Eigen::VectorXd> data_subset_1 =
      frame.getKeypointScoresOfType(0);
  const Eigen::VectorBlock<const Eigen::VectorXd> data_subset_2 =
      frame.getKeypointScoresOfType(1);
  const Eigen::VectorBlock<const Eigen::VectorXd> data_subset_3 =
      frame.getKeypointScoresOfType(2);

  EXPECT_TRUE(EIGEN_MATRIX_NEAR(data, data_subset_1, 1e-6));
  EXPECT_TRUE(EIGEN_MATRIX_NEAR(data_zero, data_subset_2, 1e-6));
  EXPECT_TRUE(EIGEN_MATRIX_NEAR(data_ext, data_subset_3, 1e-6));

  for (int i = 0; i < 10; i++) {
    const double ref = frame.getKeypointScoreOfType(i, 0);
    EXPECT_EQ(data(i), ref);
  }

  for (int i = 0; i < 10; i++) {
    const double ref = frame.getKeypointScoreOfType(i, 1);
    EXPECT_EQ(data_zero(i), ref);
  }

  for (int i = 0; i < 20; i++) {
    const double ref = frame.getKeypointScoreOfType(i, 2);
    EXPECT_EQ(data_ext(i), ref);
  }

  // Check memory addresses again and that no copy operations happened
  CHECK_EQ(&(data_3.coeff(0)), &(data_subset_1.coeff(0)));
  CHECK_EQ(&(data_3.coeff(10)), &(data_subset_2.coeff(0)));
  CHECK_EQ(&(data_3.coeff(20)), &(data_subset_3.coeff(0)));
}

TEST(Frame, SetGetTrackIds) {
  aslam::VisualFrame frame;
  Eigen::VectorXi data;
  data.resize(10);
  data.setRandom();
  frame.setTrackIds(data);
  const Eigen::VectorXi& data_2 = frame.getTrackIds();
  EXPECT_TRUE(EIGEN_MATRIX_EQUAL(data, data_2));
  EXPECT_EQ(&data_2, frame.getTrackIdsMutable());
  for (int i = 0; i < data.cols(); ++i) {
    int ref = frame.getTrackId(i);
    EXPECT_EQ(data(i), ref);
  }

  // Test extending as well as the default padding
  Eigen::Matrix2Xd keypoints(2, 40);
  keypoints.setRandom();
  frame.setKeypointMeasurements(keypoints);

  Eigen::VectorXi data_ext(20);
  data_ext.setRandom();
  frame.extendTrackIds(data_ext, -1);
  const Eigen::VectorXi& data_3 = frame.getTrackIds();
  EXPECT_EQ(data_3.size(), keypoints.cols());

  Eigen::VectorXi data_zero(10);
  data_zero.setConstant(-1);
  EXPECT_TRUE(EIGEN_MATRIX_EQUAL(data, data_3.segment(0, 10)));
  EXPECT_TRUE(EIGEN_MATRIX_EQUAL(data_zero, data_3.segment(10, 10)));
  EXPECT_TRUE(EIGEN_MATRIX_EQUAL(data_ext, data_3.segment(20, 20)));

  // Some descriptors are necessary in the frame since the keypoint type
  // is defined by the descriptor type
  aslam::VisualFrame::DescriptorsT desc_1(8, 10);
  aslam::VisualFrame::DescriptorsT desc_2(8, 10);
  aslam::VisualFrame::DescriptorsT desc_3(8, 20);
  desc_1.setRandom();
  desc_2.setRandom();
  desc_3.setRandom();
  frame.setDescriptors(desc_1, 0);
  frame.extendDescriptors(desc_2, 1);
  frame.extendDescriptors(desc_3, 2);

  const Eigen::VectorBlock<const Eigen::VectorXi> data_subset_1 =
      frame.getTrackIdsOfType(0);
  const Eigen::VectorBlock<const Eigen::VectorXi> data_subset_2 =
      frame.getTrackIdsOfType(1);
  const Eigen::VectorBlock<const Eigen::VectorXi> data_subset_3 =
      frame.getTrackIdsOfType(2);

  EXPECT_TRUE(EIGEN_MATRIX_EQUAL(data, data_subset_1));
  EXPECT_TRUE(EIGEN_MATRIX_EQUAL(data_zero, data_subset_2));
  EXPECT_TRUE(EIGEN_MATRIX_EQUAL(data_ext, data_subset_3));

  for (int i = 0; i < 10; i++) {
    const int ref = frame.getTrackIdOfType(i, 0);
    EXPECT_EQ(data(i), ref);
  }

  for (int i = 0; i < 10; i++) {
    const int ref = frame.getTrackIdOfType(i, 1);
    EXPECT_EQ(data_zero(i), ref);
  }

  for (int i = 0; i < 20; i++) {
    const int ref = frame.getTrackIdOfType(i, 2);
    EXPECT_EQ(data_ext(i), ref);
  }

  // Check memory addresses again and that no copy operations happened
  CHECK_EQ(&(data_3.coeff(0)), &(data_subset_1.coeff(0)));
  CHECK_EQ(&(data_3.coeff(10)), &(data_subset_2.coeff(0)));
  CHECK_EQ(&(data_3.coeff(20)), &(data_subset_3.coeff(0)));

  // Check getting mutable blocks for a certain feature type
  Eigen::VectorBlock<Eigen::VectorXi> data_subset_1_mutable = 
      frame.getTrackIdsOfTypeMutable(0);
  Eigen::VectorBlock<Eigen::VectorXi> data_subset_2_mutable = 
      frame.getTrackIdsOfTypeMutable(1);
  Eigen::VectorBlock<Eigen::VectorXi> data_subset_3_mutable = 
      frame.getTrackIdsOfTypeMutable(2);

  // Try modifying in place and check that the modifications happened
  Eigen::VectorXi data_new(40);
  data_new.setRandom();
  data_subset_1_mutable = data_new.segment(0, 10);
  data_subset_2_mutable = data_new.segment(10, 20);
  data_subset_3_mutable = data_new.segment(20, 40);
  EXPECT_TRUE(EIGEN_MATRIX_EQUAL(data_3, data_new));

  // Check memory addresses again and that no copy operations happened
  CHECK_EQ(&(data_3.coeff(0)), &(data_subset_1_mutable.coeff(0)));
  CHECK_EQ(&(data_3.coeff(10)), &(data_subset_2_mutable.coeff(0)));
  CHECK_EQ(&(data_3.coeff(20)), &(data_subset_3_mutable.coeff(0)));
}

TEST(Frame, NamedChannel) {
  aslam::VisualFrame frame;
  Eigen::VectorXd data;
  data.resize(10);
  data.setRandom();
  std::string channel_name = "test_channel";
  EXPECT_FALSE(frame.hasChannel(channel_name));
  EXPECT_DEATH(frame.getChannelData<Eigen::VectorXd>(channel_name), "^");

  frame.addChannel<Eigen::VectorXd>(channel_name);
  EXPECT_TRUE(frame.hasChannel(channel_name));
  frame.setChannelData(channel_name, data);

  const Eigen::VectorXd& data_2 =
      frame.getChannelData<Eigen::VectorXd>(channel_name);
  EXPECT_TRUE(EIGEN_MATRIX_NEAR(data, data_2, 1e-6));
}

TEST(Frame, SetGetImage) {
  aslam::VisualFrame frame;
  cv::Mat data(10,10,CV_8SC3,uint8_t(7));

  frame.setRawImage(data);
  const cv::Mat& data_2 = frame.getRawImage();
  EXPECT_TRUE(gtest_catkin::ImagesEqual(data, data_2));
}

TEST(Frame, CopyConstructor) {
  aslam::Camera::Ptr camera = aslam::PinholeCamera::createTestCamera();
  aslam::VisualFrame frame;
  frame.setCameraGeometry(camera);

  // Set timestamps.
  constexpr int64_t kTimestamp = 100;
  frame.setTimestampNanoseconds(kTimestamp);

  // Set some random Data.
  constexpr size_t kNumRandomValues = 10;
  Eigen::Matrix2Xd keypoints = Eigen::Matrix2Xd::Random(2, kNumRandomValues);
  frame.setKeypointMeasurements(keypoints);
  Eigen::VectorXd uncertainties = Eigen::VectorXd::Random(1, kNumRandomValues);
  frame.setKeypointMeasurementUncertainties(uncertainties);
  Eigen::VectorXd orientations = Eigen::VectorXd::Random(1, kNumRandomValues);
  frame.setKeypointOrientations(orientations);
  Eigen::VectorXd scores = Eigen::VectorXd::Random(1, kNumRandomValues);
  frame.setKeypointScores(scores);
  Eigen::VectorXd scales = Eigen::VectorXd::Random(1, kNumRandomValues);
  frame.setKeypointScales(scales);
  aslam::VisualFrame::DescriptorsT descriptors =
      aslam::VisualFrame::DescriptorsT::Random(384, kNumRandomValues);
  frame.setDescriptors(descriptors);
  Eigen::VectorXi track_ids = Eigen::VectorXi::Random(1, 10);
  frame.setTrackIds(track_ids);

  // Set image.
  cv::Mat image = cv::Mat(3, 2, CV_8UC1);
  cv::randu(image, cv::Scalar::all(0), cv::Scalar::all(255));
  frame.setRawImage(image);

  // Clone and compare.
  aslam::VisualFrame frame_cloned(frame);
  EXPECT_EQ(camera.get(), frame_cloned.getCameraGeometry().get());

  EXPECT_EQ(kTimestamp, frame_cloned.getTimestampNanoseconds());

  EIGEN_MATRIX_EQUAL(keypoints, frame_cloned.getKeypointMeasurements());
  EIGEN_MATRIX_EQUAL(uncertainties, frame_cloned.getKeypointMeasurementUncertainties());
  EIGEN_MATRIX_EQUAL(orientations, frame_cloned.getKeypointOrientations());
  EIGEN_MATRIX_EQUAL(scores, frame_cloned.getKeypointScores());
  EIGEN_MATRIX_EQUAL(scales, frame_cloned.getKeypointScales());
  EIGEN_MATRIX_EQUAL(descriptors, frame_cloned.getDescriptors());
  EIGEN_MATRIX_EQUAL(track_ids, frame_cloned.getTrackIds());

  EXPECT_NEAR_OPENCV(image, frame_cloned.getRawImage(), 0);

  EXPECT_TRUE(frame == frame_cloned);
}

TEST(Frame, getNormalizedBearingVectors) {
  // Create a test nframe with some keypoints.
  aslam::UnifiedProjectionCamera::Ptr camera = aslam::UnifiedProjectionCamera::createTestCamera();
  aslam::VisualFrame::Ptr frame = aslam::VisualFrame::createEmptyTestVisualFrame(camera, 0);

  const size_t kNumKeypoints = 10;
  Eigen::Matrix2Xd keypoints;
  keypoints.resize(Eigen::NoChange, kNumKeypoints);
  for (size_t idx = 0; idx < kNumKeypoints - 1; ++idx) {
    keypoints.col(idx) = camera->createRandomKeypoint();
  }
  keypoints.col(kNumKeypoints - 1) = Eigen::Vector2d(1e8, 1e8); // Add one invalid keypoint.
  frame->setKeypointMeasurements(keypoints);

  // Some descriptors are necessary in the frame since the keypoint
  // number for a certain type is defined through the descriptors
  aslam::VisualFrame::DescriptorsT desc(8, 10);
  desc.setRandom();
  frame->setDescriptors(desc);

  // Get bearing vectors.
  std::vector<size_t> keypoint_indices;
  keypoint_indices.emplace_back(1);
  keypoint_indices.emplace_back(3);
  keypoint_indices.emplace_back(2);
  keypoint_indices.emplace_back(4);
  keypoint_indices.emplace_back(kNumKeypoints - 1);  // This is the invalid keypoint.

  const int descriptor_type = 0;
  std::vector<unsigned char> projection_success;
  Eigen::Matrix3Xd bearing_vectors = frame->getNormalizedBearingVectors(
      keypoint_indices, descriptor_type, &projection_success);

  // Check by manually calculating the normalized bearing vectors.
  const size_t num_bearing_vectors = static_cast<size_t>(bearing_vectors.cols());
  ASSERT_EQ(num_bearing_vectors, keypoint_indices.size());
  ASSERT_EQ(num_bearing_vectors, projection_success.size());

  for (size_t bearing_idx = 0; bearing_idx < num_bearing_vectors; ++bearing_idx) {
    // Manually backproject.
    Eigen::Vector3d point_3d;
    bool success_manual = camera->backProject3(keypoints.col(keypoint_indices[bearing_idx]),
                                               &point_3d);
    Eigen::Vector3d bearing_vector_manual = point_3d.normalized();

    Eigen::Vector3d bearing_vector = bearing_vectors.col(bearing_idx);
    EXPECT_TRUE(EIGEN_MATRIX_NEAR(bearing_vector, bearing_vector_manual, 1e-12));
    EXPECT_EQ(success_manual, projection_success[bearing_idx]);
  }
}

ASLAM_UNITTEST_ENTRYPOINT
