#include <gtest/gtest.h>
#include <maplab-common/test/testing-entrypoint.h>
#include <maplab-common/test/testing-predicates.h>
#include "localization-fusion/localization-filter.h"
#include "localization-fusion/ukf.h"

/// To expose the filter for access in the unittest methods
class UkfPassThrough2d : public maplab::LocalizationFilter<maplab::Ukf> {
 public:
  UkfPassThrough2d() : maplab::LocalizationFilter<maplab::Ukf>() {}

  maplab::Ukf& getFilter() {
    return filter_;
  }

  void setUpdateVectorTo2d() {
    update_vector_[maplab::localization_fusion::StateMemberX] = 1;
    update_vector_[maplab::localization_fusion::StateMemberY] = 1;
    update_vector_[maplab::localization_fusion::StateMemberZ] = 0;
    update_vector_[maplab::localization_fusion::StateMemberRoll] = 0;
    update_vector_[maplab::localization_fusion::StateMemberPitch] = 0;
    update_vector_[maplab::localization_fusion::StateMemberYaw] = 0;
  }
};

class UKFLocalizationTest2d : public ::testing::Test {
 public:
  virtual void SetUp() {
    localization_fuser_.reset();
    localization_fuser_.setUpdateVectorTo2d();

    // fill buffer from data, get ground truth values as well
    LOG(INFO) << "Loading odometry data.";
    const std::string odom_file_name =
        "./localization_fusion_test_data/odom-estimate.txt";
    std::ifstream in_stream(odom_file_name.c_str(), std::ifstream::in);
    std::string line;
    std::vector<double> odom_errs;

    while (getline(in_stream, line)) {
      std::istringstream iss(line);

      double x, y, x_gt_odom, y_gt_odom, timestamp_ms;
      iss >> timestamp_ms;
      iss >> x;
      iss >> y;
      iss >> x_gt_odom;
      iss >> y_gt_odom;
      int64_t timestamp_ns = aslam::time::from_microseconds(timestamp_ms);
      double err = sqrt(
          (x_gt_odom - x) * (x_gt_odom - x) +
          (y_gt_odom - y) * (y_gt_odom - y));
      odom_errs.push_back(err);
      addViNodeToBuffer(timestamp_ns, x, y);
    }
    odom_rms = std::accumulate(odom_errs.begin(), odom_errs.end(), 0.0) /
               static_cast<double>(odom_errs.size());
    in_stream.close();

    // get lidar measurements and ground truth data
    LOG(INFO) << "Loading lidar measurement data.";
    const std::string lidar_file_name =
        "./localization_fusion_test_data/lidar-localizations.txt";
    std::ifstream lidar_in_stream(lidar_file_name.c_str(), std::ifstream::in);

    Eigen::MatrixXd covariance_lidar = Eigen::MatrixXd::Zero(
        maplab::localization_fusion::STATE_SIZE,
        maplab::localization_fusion::STATE_SIZE);
    covariance_lidar.operator()(0, 0) = 0.0225;
    covariance_lidar.operator()(1, 1) = 0.0225;

    while (getline(lidar_in_stream, line)) {
      std::istringstream iss(line);
      double x, y, x_gt, y_gt, timestamp_ms;
      iss >> timestamp_ms;
      iss >> x;
      iss >> y;
      iss >> x_gt;
      iss >> y_gt;
      int64_t timestamp_ns = aslam::time::from_microseconds(timestamp_ms);
      // wrap ground truth into Measurement
      Eigen::VectorXd measurement =
          Eigen::VectorXd::Zero(maplab::localization_fusion::STATE_SIZE);
      measurement[maplab::localization_fusion::StateMemberX] = x_gt;
      measurement[maplab::localization_fusion::StateMemberY] = y_gt;

      maplab::LocalizationFilterMeasurement gt_meas;
      gt_meas.time_ = timestamp_ns;
      gt_meas.measurement_ = measurement;
      ground_truth_data.push_back(gt_meas);

      // wrap lidar measurement into ML format
      common::LocalizationResult lidar_measurement;
      lidar_measurement.timestamp_ns = timestamp_ns;
      lidar_measurement.T_G_B.getPosition() = kindr::minimal::Position(x, y, 0);
      lidar_measurement.T_G_M.setIdentity();
      lidar_measurement.T_G_B_covariance = covariance_lidar;
      lidar_data.push_back(lidar_measurement);
    }
    lidar_in_stream.close();
  }
  UKFLocalizationTest2d()
      : buffer_T_M_B_(
            aslam::time::seconds(100u), aslam::time::milliseconds(500)),
        localization_fuser_() {
    odometry_covariance_per_s_ = Eigen::Matrix<double, 6, 6>::Zero();
    // clang-format off
    odometry_covariance_per_s_ << 0.05, 0.00, 0.00, 0.00, 0.00, 0.00,
                                  0.00, 0.05, 0.00, 0.00, 0.00, 0.00,
                                  0.00, 0.00, 0.06, 0.00, 0.00, 0.00,
                                  0.00, 0.00, 0.00, 0.03, 0.00, 0.00,
                                  0.00, 0.00, 0.00, 0.00, 0.03, 0.00,
                                  0.00, 0.00, 0.00, 0.00, 0.00, 0.06;
    // clang-format on
  }

 protected:
  void addViNodeToBuffer(
      const int64_t timestamp_ns, const double x, const double y) {
    vio::ViNodeState vi_node;
    vi_node.set_T_M_I(
        aslam::Transformation(aslam::Position3D(x, y, 0), aslam::Quaternion()));
    vi_node.set_v_M_I(Eigen::Vector3d(0, 0, 0));
    vi_node.setAccBias(Eigen::Vector3d(0, 0, 0));
    vi_node.setGyroBias(Eigen::Vector3d(0, 0, 0));
    vi_node.setTimestamp(timestamp_ns);
    buffer_T_M_B_.bufferOdometryEstimate(vi_node);
  }
  aslam::TransformationCovariance odometry_covariance_per_s_;
  std::vector<maplab::LocalizationFilterMeasurement> ground_truth_data;
  std::vector<common::LocalizationResult> lidar_data;

  double odom_rms;

  vio_common::PoseLookupBuffer buffer_T_M_B_;

  UkfPassThrough2d localization_fuser_;
};

TEST_F(UKFLocalizationTest2d, real_data_2d) {
  vio::ViNodeState T_M_B;
  std::vector<double> rms;
  LOG(INFO) << "Starting testing.";
  for (size_t i = 0; i < lidar_data.size(); ++i) {
    const common::LocalizationResult loc = lidar_data[i];
    if (buffer_T_M_B_.interpolateViNodeStateAt(loc.timestamp_ns, &T_M_B) ==
        vio_common::PoseLookupBuffer::ResultStatus::kSuccessInterpolated) {
      if (localization_fuser_.getFilter().getInitializedStatus()) {
        double delta_s = aslam::time::to_seconds(
            loc.timestamp_ns - localization_fuser_.getLastMessageTimeNs());
        aslam::TransformationCovariance odometry_covariance;
        odometry_covariance = odometry_covariance_per_s_ * delta_s;
        T_M_B.setPoseCovariance(odometry_covariance);
        Eigen::VectorXd measurement;
        localization_fuser_.localizationCallback(loc, T_M_B);
        measurement = localization_fuser_.getFilter().getState();
        const double rms_measurement =
            (ground_truth_data[i].measurement_ - measurement).head(2).norm();
        rms.push_back(rms_measurement);
      } else {
        localization_fuser_.initialize(loc, T_M_B.get_T_M_I());
      }
    }
  }
  // We clearly expect the filter to outperform the odometry prior.
  const double rms_error = std::accumulate(rms.begin(), rms.end(), 0.0) /
                           static_cast<double>(rms.size());
  LOG(INFO) << "rms_filter: " << rms_error << " vs. odometry_rms: " << odom_rms;
  ASSERT_LT(rms_error, odom_rms);
}

MAPLAB_UNITTEST_ENTRYPOINT
