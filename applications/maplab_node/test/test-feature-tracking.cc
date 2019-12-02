#include <chrono>
#include <condition_variable>
#include <thread>

#include <aslam/cameras/random-camera-generator.h>
#include <gtest/gtest.h>
#include <maplab-common/test/testing-entrypoint.h>
#include <maplab-common/test/testing-predicates.h>

#include "maplab-node/feature-tracking.h"
#include "maplab-node/synchronizer.h"

namespace maplab {

class VioPipelineTest : public ::testing::Test {
 public:
  void dataCallback(const vio::SynchronizedNFrame::Ptr& nframe_imu) {
    if (pipeline_->trackSynchronizedNFrameCallback(nframe_imu)) {
      std::lock_guard<std::mutex> lock(m_num_output_frames_);
      ++num_output_frames_;

      if (num_output_frames_ == kNumExpectedFrames) {
        work_really_done_ = true;
        cv_work_done_.notify_all();
      }
    }
  }

 protected:
  virtual void SetUp() {
    ncamera_ = aslam::createTestNCamera(kNumCameras);
    sensor_manager_ =
        std::unique_ptr<vi_map::SensorManager>(new vi_map::SensorManager());
    synchronizer_.reset(new Synchronizer(*sensor_manager_));
    synchronizer_->initializeNCameraSynchronization(ncamera_);
    pipeline_.reset(
        new FeatureTracking(ncamera_, synchronizer_->T_M_B_buffer()));
    num_output_frames_ = 0;
    work_really_done_ = false;
  }

  Synchronizer::Ptr synchronizer_;
  FeatureTracking::Ptr pipeline_;
  aslam::NCamera::Ptr ncamera_;
  static constexpr size_t kNumCameras = 2u;

  int num_output_frames_;
  std::mutex m_num_output_frames_;

  static constexpr int kNumFeededFrames = 20;
  // Expected frames are feeded frames minus one at the beginning and one to
  // populate the previous frame.
  static constexpr int kNumExpectedFrames = kNumFeededFrames - 2;

  static constexpr size_t kGyroBiasYOffset = 1000u;

  std::condition_variable cv_work_done_;
  bool work_really_done_;
  std::unique_ptr<vi_map::SensorManager> sensor_manager_;

 private:
  vi_map::Imu imu_sensor_;
};

constexpr int VioPipelineTest::kNumFeededFrames;
constexpr int VioPipelineTest::kNumExpectedFrames;
constexpr size_t VioPipelineTest::kGyroBiasYOffset;

TEST_F(VioPipelineTest, PipelineWorks) {
  std::function<void(const vio::SynchronizedNFrame::Ptr&)> callback =
      std::bind(&VioPipelineTest::dataCallback, this, std::placeholders::_1);
  synchronizer_->registerSynchronizedNFrameCallback(callback);

  constexpr int64_t kTimestepNs = 1e9;

  for (int64_t t = 0; t < kNumFeededFrames * kTimestepNs; t += kTimestepNs) {
    for (size_t i = 0; i < kNumCameras; ++i) {
      cv::Mat image = cv::Mat(
          ncamera_->getCamera(i).imageHeight(),
          ncamera_->getCamera(i).imageWidth(), CV_8UC1);
      cv::randu(image, cv::Scalar::all(0), cv::Scalar::all(255));

      synchronizer_->processCameraImage(i, image, t);
    }

    Eigen::Matrix<int64_t, 1, Eigen::Dynamic> timestamps_nanoseconds(1, 4);
    timestamps_nanoseconds << t, t + 0.5 * kTimestepNs, t + kTimestepNs - 5,
        t + kTimestepNs - 1;

    Eigen::Matrix<double, 6, Eigen::Dynamic> imu_measurements;
    imu_measurements.resize(Eigen::NoChange, 4);
    imu_measurements.setRandom();

    synchronizer_->processImuMeasurements(
        timestamps_nanoseconds, imu_measurements);
    vio::ViNodeState vi_node;
    vi_node.set_T_M_I(
        aslam::Transformation(aslam::Position3D(t, 0, 0), aslam::Quaternion()));
    vi_node.set_v_M_I(Eigen::Vector3d(0, t, 0));
    vi_node.setAccBias(Eigen::Vector3d(0, 0, t));
    vi_node.setGyroBias(Eigen::Vector3d(t, kGyroBiasYOffset + t, 0));
    vi_node.setTimestamp(t);
    synchronizer_->processOdometryMeasurement(vi_node);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  {
    std::unique_lock<std::mutex> lock(m_num_output_frames_);
    // Wait till we are done, but not longer than few seconds.
    cv_work_done_.wait_for(
        lock, std::chrono::seconds(10), [this]() { return work_really_done_; });

    EXPECT_EQ(num_output_frames_, kNumExpectedFrames);
  }

  synchronizer_->shutdown();
}

}  // namespace maplab

MAPLAB_UNITTEST_ENTRYPOINT
