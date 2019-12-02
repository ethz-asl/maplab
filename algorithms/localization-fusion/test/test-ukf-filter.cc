#include <cmath>
#include <eigen-checks/glog.h>
#include <eigen-checks/gtest.h>
#include <gtest/gtest.h>
#include <maplab-common/test/testing-entrypoint.h>
#include <maplab-common/test/testing-predicates.h>
#include "localization-fusion/localization-filter.h"
#include "localization-fusion/ukf.h"

// https://stackoverflow.com/questions/7115459/c-rand-and-srand-gets-different-output-on-different-machines
static unsigned long int next = 1;  // NOLINT

int test_rand(void) {
  // RAND_MAX assumed to be 32767
  next = next * 1103515245 + 12345;
  return (unsigned int)(next / 65536) % 32768;
}

void test_srand(unsigned int seed) {
  next = seed;
}

double test_rand_double(void) {
  return static_cast<double>(test_rand()) / static_cast<double>(RAND_MAX);
}

class UkfPassThrough : public maplab::LocalizationFilter<maplab::Ukf> {
 public:
  UkfPassThrough() : maplab::LocalizationFilter<maplab::Ukf>() {}

  maplab::Ukf& getFilter() {
    return filter_;
  }

  void processMeasurement(
      const maplab::LocalizationFilterMeasurement& measurement,
      const maplab::LocalizationFilterPrediction& odom_prediction) {
    filter_.processMeasurement(measurement, odom_prediction, update_vector_);
  }

  void setMahalanobisThreshold(double threshold) {
    filter_.setMahalanobisThreshold(threshold);
  }
};

class UKFLocalizationTest : public ::testing::Test {
 public:
  UKFLocalizationTest()
      : buffer_T_M_B_(
            aslam::time::seconds(30u), aslam::time::milliseconds(500)),
        localization_fuser_() {}

 protected:
  virtual void SetUp() {
    localization_fuser_.reset();
    // fill buffer with random stuff
    for (int64_t timestamp_ms = 0; timestamp_ms <= kMaxViNodeTimestampMs;
         ++timestamp_ms) {
      int64_t timestamp_ns = timestamp_ms * 1e6;
      addViNodeToBuffer(timestamp_ns);
      test_srand(42);
    }
  }

  maplab::Ukf& getFilter() {
    return localization_fuser_.getFilter();
  }

  void addViNodeToBuffer(const int64_t timestamp_ns) {
    vio::ViNodeState vi_node;
    vi_node.set_T_M_I(aslam::Transformation(
        aslam::Position3D(timestamp_ns, 0, 0), aslam::Quaternion()));
    vi_node.set_v_M_I(Eigen::Vector3d(0, timestamp_ns, 0));
    vi_node.setAccBias(Eigen::Vector3d(0, 0, timestamp_ns));
    vi_node.setGyroBias(
        Eigen::Vector3d(timestamp_ns, kGyroBiasYOffset + timestamp_ns, 0));
    vi_node.setTimestamp(timestamp_ns);
    buffer_T_M_B_.bufferOdometryEstimate(vi_node);
  }

  vio_common::PoseLookupBuffer buffer_T_M_B_;

  UkfPassThrough localization_fuser_;

 private:
  static constexpr int64_t kMaxViNodeTimestampMs = 500;
  static constexpr size_t kGyroBiasYOffset = 1000u;
};

TEST_F(UKFLocalizationTest, rpy_to_from) {
  Eigen::Vector3d t(0, 0, 0);
  for (int i = 0; i < 10000; ++i) {
    Eigen::Quaterniond q, q_random;
    q_random.x() = test_rand_double();
    q_random.y() = test_rand_double();
    q_random.z() = test_rand_double();
    q_random.w() = test_rand_double();
    q_random.normalize();

    aslam::Transformation tmp(q_random, t);
    double y, p, r;
    maplab::matrixToRPY(tmp.getRotationMatrix().eval(), y, p, r);
    maplab::RPYtoQuaternion(r, p, y, q);

    // if everything is correct, this should be a unit quaternion
    const Eigen::Quaterniond q2 = q_random.conjugate() * q;

    ASSERT_NEAR(std::abs(q2.w()), 1.0, 1E-4);
    ASSERT_NEAR(q2.x(), 0.0, 1E-4);
    ASSERT_NEAR(q2.y(), 0.0, 1E-4);
    ASSERT_NEAR(q2.z(), 0.0, 1E-4);
  }
}

TEST_F(UKFLocalizationTest, accept_valid_time) {
  aslam::Transformation tmp;
  common::LocalizationResult result;
  result.timestamp_ns = aslam::time::milliseconds(200);
  result.T_G_M = aslam::Transformation();
  result.T_G_B = aslam::Transformation();
  localization_fuser_.initialize(result, tmp);
  ASSERT_TRUE(
      aslam::time::isValidTime(localization_fuser_.getLastMessageTimeNs()));
}

TEST_F(UKFLocalizationTest, reject_early_time) {
  aslam::Transformation tmp;
  int64_t expected_time_ns = aslam::time::milliseconds(200);
  common::LocalizationResult result;
  result.timestamp_ns = expected_time_ns;
  result.T_G_B = aslam::Transformation();
  result.T_G_M = aslam::Transformation();
  localization_fuser_.initialize(result, tmp);

  result.timestamp_ns = aslam::time::milliseconds(150);
  vio::ViNodeState tmp_state;
  localization_fuser_.localizationCallback(result, tmp_state);
  ASSERT_EQ(localization_fuser_.getLastMessageTimeNs(), expected_time_ns);
}

TEST_F(UKFLocalizationTest, measurement_handling) {
  Eigen::MatrixXd initialCovar(
      maplab::localization_fusion::STATE_SIZE,
      maplab::localization_fusion::STATE_SIZE);
  initialCovar.setIdentity();
  initialCovar *= 0.5;
  getFilter().setEstimateErrorCovariance(initialCovar);

  EXPECT_TRUE(EIGEN_MATRIX_EQUAL(
      getFilter().getEstimateErrorCovariance(), initialCovar));

  Eigen::VectorXd measurement(maplab::localization_fusion::STATE_SIZE);
  for (size_t i = 0; i < maplab::localization_fusion::STATE_SIZE; ++i) {
    measurement[i] = i * 0.01 * maplab::localization_fusion::STATE_SIZE;
  }

  Eigen::MatrixXd measurementCovariance(
      maplab::localization_fusion::STATE_SIZE,
      maplab::localization_fusion::STATE_SIZE);
  measurementCovariance.setIdentity();
  for (size_t i = 0; i < maplab::localization_fusion::STATE_SIZE; ++i) {
    measurementCovariance(i, i) = 1e-9;
  }

  maplab::LocalizationFilterMeasurement pose_measurement_M_B;
  maplab::LocalizationFilterPrediction odom_tmp;
  pose_measurement_M_B.measurement_ = measurement;
  pose_measurement_M_B.covariance_ = measurementCovariance;
  pose_measurement_M_B.time_ = aslam::time::seconds(1000);

  // initializing the filter
  localization_fuser_.processMeasurement(pose_measurement_M_B, odom_tmp);

  EXPECT_TRUE(EIGEN_MATRIX_EQUAL(getFilter().getState(), measurement));
  EXPECT_TRUE(EIGEN_MATRIX_EQUAL(
      getFilter().getEstimateErrorCovariance(), measurementCovariance));

  getFilter().setEstimateErrorCovariance(initialCovar);

  // Now fuse another measurement and check the output.
  // We know what the filter's state should be when
  // this is complete, so we'll check the difference and
  // make sure it's suitably small.
  Eigen::VectorXd measurement2 = measurement;

  measurement2 *= 2.0;

  for (size_t i = 0; i < maplab::localization_fusion::STATE_SIZE; ++i) {
    measurementCovariance(i, i) = 1e-9;
  }

  maplab::LocalizationFilterMeasurement pose_measurement_M_B2;
  maplab::LocalizationFilterPrediction odom_tmp2;
  pose_measurement_M_B2.measurement_ = measurement2;
  pose_measurement_M_B2.covariance_ = measurementCovariance;
  pose_measurement_M_B2.time_ = aslam::time::seconds(1002);

  localization_fuser_.setMahalanobisThreshold(
      std::numeric_limits<double>::max());
  // this prediction should be ignored given the large mahalanobis threshold
  odom_tmp2.predicted_state_ =
      Eigen::VectorXd::Zero(maplab::localization_fusion::STATE_SIZE);
  odom_tmp2.predicted_state_ << 5.0687, 8.3323, 4.3098, 6.0756, 1.866, 5.9314;
  odom_tmp2.covariance_ = Eigen::MatrixXd::Zero(
      maplab::localization_fusion::STATE_SIZE,
      maplab::localization_fusion::STATE_SIZE);
  // clang-format off
  odom_tmp2.covariance_ <<  0.10, 0.00, 0.00, 0.00, 0.00, 0.00,
                            0.00, 0.10, 0.00, 0.00, 0.00, 0.00,
                            0.00, 0.00, 0.12, 0.00, 0.00, 0.00,
                            0.00, 0.00, 0.00, 0.06, 0.00, 0.00,
                            0.00, 0.00, 0.00, 0.00, 0.06, 0.00,
                            0.00, 0.00, 0.00, 0.00, 0.00, 0.12;
  // clang-format on
  localization_fuser_.processMeasurement(pose_measurement_M_B2, odom_tmp2);

  Eigen::VectorXd measurement_err =
      measurement2.eval() - getFilter().getState();
  for (size_t i = 0; i < maplab::localization_fusion::STATE_SIZE; ++i) {
    EXPECT_LT(::fabs(measurement_err[i]), 0.001);
  }
}

MAPLAB_UNITTEST_ENTRYPOINT
