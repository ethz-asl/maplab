#ifndef MAP_OPTIMIZATION_OPTIMIZATION_TERMS_ADDITION_H_
#define MAP_OPTIMIZATION_OPTIMIZATION_TERMS_ADDITION_H_

#include "map-optimization/optimization-problem.h"

#include <memory>

namespace map_optimization {

int addLandmarkTerms(
    const vi_map::FeatureType use_feature_type,
    const bool fix_landmark_positions, const bool fix_intrinsics,
    const bool fix_extrinsics_rotation, const bool fix_extrinsics_translation,
    const size_t min_landmarks_per_frame, OptimizationProblem* problem);

int addLandmarkTermsForVertices(
    const vi_map::FeatureType use_feature_type,
    const bool fix_landmark_positions, const bool fix_intrinsics,
    const bool fix_extrinsics_rotation, const bool fix_extrinsics_translation,
    const size_t min_landmarks_per_frame,
    const std::shared_ptr<ceres::LocalParameterization>& pose_parameterization,
    const std::shared_ptr<ceres::LocalParameterization>&
        baseframe_parameterization,
    const std::shared_ptr<ceres::LocalParameterization>&
        camera_parameterization,
    const pose_graph::VertexIdList& vertices, OptimizationProblem* problem);

void addLandmarkTermForKeypoint(
    const vi_map::FeatureType feature_type, const int keypoint_idx,
    const int frame_idx, const bool fix_landmark_positions,
    const bool fix_intrinsics, const bool fix_extrinsics_rotation,
    const bool fix_extrinsics_translation,
    const std::shared_ptr<ceres::LocalParameterization>& pose_parameterization,
    const std::shared_ptr<ceres::LocalParameterization>&
        baseframe_parameterization,
    const std::shared_ptr<ceres::LocalParameterization>&
        camera_parameterization,
    vi_map::Vertex* vertex_ptr, OptimizationProblem* problem);

int addInertialTerms(
    const bool fix_gyro_bias, const bool fix_accel_bias,
    const bool fix_velocity, const double gravity_magnitude,
    OptimizationProblem* problem);

int addInertialTermsForEdges(
    const bool fix_gyro_bias, const bool fix_accel_bias,
    const bool fix_velocity, const double gravity_magnitude,
    const aslam::SensorId& imu_sensor_id,
    const std::shared_ptr<ceres::LocalParameterization>& pose_parameterization,
    const pose_graph::EdgeIdList& edges, OptimizationProblem* problem);

int addWheelOdometryTerms(
    const bool fix_extrinsics, OptimizationProblem* problem);

int add6DoFOdometryTerms(
    const bool fix_extrinsics, OptimizationProblem* problem);

int addRelativePoseTermsForEdges(
    const vi_map::Edge::EdgeType edge_type,
    const pose_graph::EdgeIdList& provided_edges, const bool fix_extrinsics,
    const std::shared_ptr<ceres::LocalParameterization>& pose_parameterization,
    const std::shared_ptr<ceres::LocalParameterization>&
        quaternion_parameterization,
    OptimizationProblem* problem);

int addAbsolutePoseConstraintsTerms(
    const bool fix_extrinsics, OptimizationProblem* problem);

int addAbsolutePoseConstraintTermsForVertices(
    const bool fix_extrinsics,
    const std::shared_ptr<ceres::LocalParameterization>& pose_parameterization,
    const std::shared_ptr<ceres::LocalParameterization>&
        baseframe_parameterization,
    const vi_map::MissionId& mission_id,
    const pose_graph::VertexIdList& vertices, OptimizationProblem* problem);

}  // namespace map_optimization

#endif  // MAP_OPTIMIZATION_OPTIMIZATION_TERMS_ADDITION_H_
