#include <chrono>
#include <thread>

#include <gtest/gtest.h>
#include <maplab-common/test/testing-entrypoint.h>
#include <maplab-common/test/testing-predicates.h>

#include "rovioli/feature-tracking.h"
#include "rovioli/imu-camera-synchronizer.h"

namespace rovioli {

class VioPipelineTest : public ::testing::Test {
 public:
  void dataCallback(const vio::SynchronizedNFrameImu::Ptr& nframe_imu) {
    if (pipeline_->trackSynchronizedNFrameImuCallback(nframe_imu)) {
      std::lock_guard<std::mutex> lock(m_num_output_frames_);
      ++num_output_frames_;
    }
  }

 protected:
  virtual void SetUp() {
    ncamera_ = aslam::NCamera::createTestNCamera(kNumCameras);
    synchronizer_.reset(new ImuCameraSynchronizer(ncamera_));
    pipeline_.reset(new FeatureTracking(ncamera_, imu_sensor_));
    num_output_frames_ = 0;
  }

  ImuCameraSynchronizer::Ptr synchronizer_;
  FeatureTracking::Ptr pipeline_;
  aslam::NCamera::Ptr ncamera_;
  static constexpr size_t kNumCameras = 2u;

  int num_output_frames_;
  std::mutex m_num_output_frames_;

 private:
  vi_map::Imu imu_sensor_;
};

TEST_F(VioPipelineTest, PipelineWorks) {
  std::function<void(const vio::SynchronizedNFrameImu::Ptr&)> callback =
      std::bind(&VioPipelineTest::dataCallback, this, std::placeholders::_1);
  synchronizer_->registerSynchronizedNFrameImuCallback(callback);

  constexpr int64_t kTimestepNs = 1e9;
  constexpr int64_t kNumFrames = 20;

  int feeded_frames = 0;
  for (int64_t t = 0; t < kNumFrames * kTimestepNs; t += kTimestepNs) {
    for (size_t i = 0; i < kNumCameras; ++i) {
      cv::Mat image = cv::Mat(
          ncamera_->getCamera(i).imageHeight(),
          ncamera_->getCamera(i).imageWidth(), CV_8UC1);
      cv::randu(image, cv::Scalar::all(0), cv::Scalar::all(255));

      synchronizer_->addCameraImage(i, image, t);
    }

    Eigen::Matrix<int64_t, 1, Eigen::Dynamic> timestamps_nanoseconds(1, 4);
    timestamps_nanoseconds << t, t + 0.5 * kTimestepNs, t + kTimestepNs - 5,
        t + kTimestepNs - 1;

    Eigen::Matrix<double, 6, Eigen::Dynamic> imu_measurements;
    imu_measurements.resize(Eigen::NoChange, 4);
    imu_measurements.setRandom();

    synchronizer_->addImuMeasurements(timestamps_nanoseconds, imu_measurements);

    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    ++feeded_frames;
  }

  std::this_thread::sleep_for(std::chrono::milliseconds(500));

  // Decrease the counter by frames skipped at the beginning + one to populate
  // the previous frame.
  feeded_frames -= ImuCameraSynchronizer::kFramesToSkipAtInit + 1;

  {
    std::lock_guard<std::mutex> lock(m_num_output_frames_);
    EXPECT_EQ(num_output_frames_, feeded_frames);
  }

  synchronizer_->shutdown();
}

}  // namespace rovioli

MAPLAB_UNITTEST_ENTRYPOINT
