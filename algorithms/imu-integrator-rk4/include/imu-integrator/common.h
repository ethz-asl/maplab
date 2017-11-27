#ifndef IMU_INTEGRATOR_COMMON_H_
#define IMU_INTEGRATOR_COMMON_H_

namespace imu_integrator {

static const int kStateSize = 16;
static const int kErrorStateSize = 15;

static const int kStateOrientationOffset = 0;
static const int kStateGyroBiasOffset = 4;
static const int kStateVelocityOffset = 7;
static const int kStateAccelBiasOffset = 10;
static const int kStatePositionOffset = 13;

static const int kErrorStateOrientationOffset = 0;
static const int kErrorStateGyroBiasOffset = 3;
static const int kErrorStateVelocityOffset = 6;
static const int kErrorStateAccelBiasOffset = 9;
static const int kErrorStatePositionOffset = 12;

static const int kStateOrientationBlockSize = 4;
static const int kErrorOrientationBlockSize = 3;
static const int kGyroBiasBlockSize = 3;
static const int kVelocityBlockSize = 3;
static const int kAccelBiasBlockSize = 3;
static const int kPositionBlockSize = 3;
static const int kStatePoseBlockSize = 7;

// New Eigen based error terms combine IMU biases
static const int kImuBiasBlockSize = 6;

static const int kImuReadingSize = 6;
static const int kAccelReadingOffset = 0;
static const int kGyroReadingOffset = 3;

static const double kNanoSecondsToSeconds = 1e-9;

}  // namespace imu_integrator

#endif  // IMU_INTEGRATOR_COMMON_H_
