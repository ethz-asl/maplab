#ifndef SIMULATION_VISUAL_INERTIAL_PATH_GENERATOR_H_
#define SIMULATION_VISUAL_INERTIAL_PATH_GENERATOR_H_

#include <map>
#include <random>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <aslam/common/memory.h>
#include <aslam/common/pose-types.h>
#include <mav_planning_utils/mav_state.h>
#include <mav_planning_utils/path_planning.h>
#include <sensors/imu.h>

namespace test_trajectory_gen {

enum class Path : int {
  kCircular = 1,
  kElliptical = 2,
  kRotationOnly = 3,
  kTranslationOnly = 4,
  kFromFile = 5,
  kFromCtor = 6
};

inline std::ostream& operator<<(std::ostream& out, const Path& value) {
  static std::map<Path, std::string> names;
  if (names.size() == 0) {
#define INSERT_ELEMENT(type, val) names[type::val] = #val
    INSERT_ELEMENT(Path, kCircular);
    INSERT_ELEMENT(Path, kElliptical);
    INSERT_ELEMENT(Path, kRotationOnly);
    INSERT_ELEMENT(Path, kTranslationOnly);
    INSERT_ELEMENT(Path, kFromFile);
    INSERT_ELEMENT(Path, kFromCtor);
#undef INSERT_ELEMENT
  }
  return out << names[value];
}

/// Parameters used to generate path and landmarks.
/// Note that not every parameter is active in every mode.
/// Alphabetized first by type, then by variable name.
struct PathAndLandmarkSettings {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  /// Vector of waypoints (x,y,z,yaw), can be used to pass
  /// user-defined waypoints to ctor.
  Aligned<std::vector, Eigen::Vector4d> pose_waypoints;

  /// Circle radius.
  double circle_radius_meter;
  /// Distance to keypoints.
  double distance_to_keypoints_meter;
  /// IMU noise parameters.
  vi_map::ImuSigmas imu_sigmas;
  ///
  double gravity_meter_by_second2;
  /// Kappa defining the ellipse shape according to
  /// x = (1.0 / kappa) * cos(t).
  double kappa;
  /// Lambda defining the ellipse shape according to
  /// y = (1.0 / lambda) * sin(t).
  double lambda;
  /// Landmark volume offset in x direction.
  double landmark_offset_x_meter;
  /// Landmark volume offset in y direction.
  double landmark_offset_y_meter;
  /// Landmark volume offset in z direction.
  double landmark_offset_z_meter;
  /// Landmark variance.
  double landmark_variance_meter;
  /// Landmark volume in x direction, centered around offset.
  double landmark_volume_x_meter;
  /// Landmark volume in y direction, centered around offset.
  double landmark_volume_y_meter;
  /// Landmark volume in z direction, centered around offset.
  double landmark_volume_z_meter;
  /// Number of rounds. Currently only applies for elliptical mode.
  double num_of_rounds;
  /// Sampling time in seconds.
  double sampling_time_second;
  /// Offset from the origin in x direction.
  double start_offset_x_meter;
  /// Offset from the origin in y direction.
  double start_offset_y_meter;
  /// Yaw start offset.
  double start_offset_yaw_radians;
  /// Offset from the origin in z direction. This is the constant height.
  double start_offset_z_meter;
  /// Ratio between radial and vertical standard deviation of the
  /// keypoint positions.
  double vertical_to_radial_variance_factor;
  /// IMU noise bias seed.
  size_t imu_noise_bias_seed;
  /// Landmark seed.
  size_t landmark_seed;
  /// Trajectory mode.
  test_trajectory_gen::Path mode;
  /// Number of path constraints.
  size_t num_of_path_constraints;
  /// Number of landmarks.
  size_t num_of_landmarks;
  /// List with waypoints: x y z yaw.
  std::string filename_waypoints;

  // Set the default values
  PathAndLandmarkSettings()
      : circle_radius_meter(10.0),
        distance_to_keypoints_meter(5.0),
        imu_sigmas(0.0, 0.0, 0.0, 0.0),
        gravity_meter_by_second2(9.81),
        kappa(0.1),
        lambda(0.1),
        landmark_offset_x_meter(50.0),
        landmark_offset_y_meter(0.0),
        landmark_offset_z_meter(0.5),
        landmark_variance_meter(3.0),
        landmark_volume_x_meter(10.0),
        landmark_volume_y_meter(10.0),
        landmark_volume_z_meter(5.0),
        num_of_rounds(1.0),
        sampling_time_second(0.1),
        start_offset_x_meter(0.0),
        start_offset_y_meter(0.0),
        start_offset_yaw_radians(0.0),
        start_offset_z_meter(0.0),
        vertical_to_radial_variance_factor(2.0),
        imu_noise_bias_seed(10),
        landmark_seed(5),
        mode(Path::kCircular),
        num_of_path_constraints(8),
        num_of_landmarks(1000),
        filename_waypoints("waypoints_translation_only.txt") {}
};

class VisualInertialPathGenerator {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  explicit VisualInertialPathGenerator(const PathAndLandmarkSettings& settings);

  virtual ~VisualInertialPathGenerator() {}

  const Eigen::Matrix3Xd& getTruePositions() const;
  const Eigen::Matrix4Xd& getTrueRotations() const;
  const Eigen::Matrix3Xd& getTrueVelocities() const;
  const Eigen::Matrix3Xd& getTrueAccBias() const;
  const Eigen::Matrix3Xd& getTrueGyroBias() const;
  const Eigen::Matrix3Xd& getLandmarks() const;
  const std::vector<uint64_t> getDescriptors() const;
  const mav_planning_utils::Motion4D<5, 2>::Vector& getPathData() const;
  const Eigen::Matrix<double, 6, Eigen::Dynamic>& getImuData() const;
  const Eigen::VectorXd& getTimestampsInSeconds() const;

  void getGroundTruthTransformations(aslam::TransformationVector* T_G_Bs) const;

  virtual void generatePath() = 0;
  virtual void generateLandmarks() = 0;

  void motionVectorToMavState(
      const mav_planning_utils::Motion4D<5, 2>& data,
      mav_planning_utils::MavState* mc_state) const;

  bool isPathGenerated() const;
  bool areLandmarksGenerated() const;

 protected:
  void motionVectorToImuData(uint_fast32_t seed);

  void getRandomVector3d(
      std::mt19937* generator, std::normal_distribution<>* distribution,
      Eigen::Vector3d* vector);

  /// \brief Sample timestamps in seconds.
  Eigen::VectorXd timestamps_seconds_;

  Eigen::Matrix3Xd G_landmarks_;

  Eigen::Matrix3Xd true_p_G_B_;
  Eigen::Matrix4Xd true_q_B_G_;
  /// Velocity of the body-frame expressed in the global frame.
  Eigen::Matrix3Xd true_G_v_B_;
  Eigen::Matrix3Xd true_acc_bias_;
  Eigen::Matrix3Xd true_gyro_bias_;

  Eigen::Matrix<double, 6, Eigen::Dynamic> imu_data_;
  mav_planning_utils::Motion4D<5, 2>::Vector path_data_;

  bool are_landmarks_generated_;
  bool is_path_generated_;

 private:
  PathAndLandmarkSettings settings_;
};

}  // namespace test_trajectory_gen

#endif  // SIMULATION_VISUAL_INERTIAL_PATH_GENERATOR_H_
