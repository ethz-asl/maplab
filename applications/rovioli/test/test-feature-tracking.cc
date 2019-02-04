#include <chrono>
#include <condition_variable>
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

      if (num_output_frames_ == kNumExpectedFrames) {
        work_really_done_ = true;
        cv_work_done_.notify_all();
      }
    }
  }

 protected:
  virtual void SetUp() {
    ncamera_ = aslam::NCamera::createTestNCamera(kNumCameras);
    synchronizer_.reset(new ImuCameraSynchronizer(ncamera_));
    pipeline_.reset(new FeatureTracking(ncamera_, imu_sensor_));
    num_output_frames_ = 0;
    work_really_done_ = false;
  }

  ImuCameraSynchronizer::Ptr synchronizer_;
  FeatureTracking::Ptr pipeline_;
  aslam::NCamera::Ptr ncamera_;
  static constexpr size_t kNumCameras = 2u;

  int num_output_frames_;
  std::mutex m_num_output_frames_;

  static constexpr int kNumFeededFrames = 20;
  // Expected frames are feeded frames minus one at the beginning and one to
  // populate the previous frame.
  static constexpr int kNumExpectedFrames =
      kNumFeededFrames - ImuCameraSynchronizer::kFramesToSkipAtInit - 1;

  std::condition_variable cv_work_done_;
  bool work_really_done_;

 private:
  vi_map::Imu imu_sensor_;
};

constexpr int VioPipelineTest::kNumFeededFrames;
constexpr int VioPipelineTest::kNumExpectedFrames;

TEST_F(VioPipelineTest, PipelineWorks) {
  std::function<void(const vio::SynchronizedNFrameImu::Ptr&)> callback =
      std::bind(&VioPipelineTest::dataCallback, this, std::placeholders::_1);
  synchronizer_->registerSynchronizedNFrameImuCallback(callback);

  constexpr int64_t kTimestepNs = 1e9;

  for (int64_t t = 0; t < kNumFeededFrames * kTimestepNs; t += kTimestepNs) {
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

}  // namespace rovioli

MAPLAB_UNITTEST_ENTRYPOINT
