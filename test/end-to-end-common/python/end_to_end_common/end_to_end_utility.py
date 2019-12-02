#!/usr/bin/env python

from __future__ import print_function

import bisect
import os

import numpy as np

from hand_eye_calibration.dual_quaternion import DualQuaternion
from hand_eye_calibration.extrinsic_calibration import ExtrinsicCalibration
from hand_eye_calibration.quaternion import Quaternion
import hand_eye_calibration.time_alignment as time_alignment

from end_to_end_common.dataset_loader import csv_row_data_to_transformation
from end_to_end_common.umeyama import umeyama


def get_cumulative_trajectory_length_for_each_point(trajectory):
    """Returns a list of trajectory lengths up to each point in the trajectory.
    """
    assert trajectory.shape[1] == 3
    num_points = trajectory.shape[0]
    trajectory_length = np.zeros(num_points)
    for row_idx in range(1, num_points):
        delta = np.linalg.norm(trajectory[row_idx, :] -
                               trajectory[row_idx - 1, :])
        trajectory_length[row_idx] = trajectory_length[row_idx - 1] + delta
    return trajectory_length


def apply_hand_eye_calibration(trajectory_G_I, marker_calibration_file):
    """Returns the transformed estimator trajectory.

    Computes the trajectory of the marker frame, given the trajectory if the eye
    and the hand_eye_calibration output.
    """
    assert os.path.isfile(marker_calibration_file)
    # Frames of reference: M = marker (hand), I = IMU (eye).
    dq_I_M = ExtrinsicCalibration.fromJson(
        marker_calibration_file).pose_dq.inverse()
    trajectory_G_M = trajectory_G_I
    for idx in range(trajectory_G_M.shape[0]):
        dq_G_I = DualQuaternion.from_pose_vector(trajectory_G_I[idx, 1:8])
        dq_G_M = dq_G_I * dq_I_M
        trajectory_G_M[idx, 1:8] = dq_G_M.to_pose()
    return trajectory_G_M


def align_datasets(estimator_data_unaligned_G_I,
                   ground_truth_data_unaligned_W_M,
                   estimate_scale,
                   align_data_to_use_meters=-1,
                   time_offset=0.0,
                   marker_calibration_file=None):
    """Sample and align the estimator trajectory to the ground truth trajectory.

    Input:
      - estimator_data_unaligned_G_I: Unaligned estimator trajectory expressed
            from global frame G to IMU frame I.
      - ground_truth_data_unaligned_W_M: Unaligned ground truth data expressed
            from ground truth world frame W (e.g. Vicon frame) to marker frame M
            (e.g. Vicon marker). M and I should be at the same position at a
            given time.
      - estimate_scale: Estimate a scale factor in addition to the transform
            and apply it to the aligned estimator trajectory.
      - align_data_to_use_meters: Only use data points up to a total data length
            as given by this parameter. If a negative value is given, the
            entire trajectory is used for the alignment.
      - time_offset: Indicates the time offset (in seconds) between the data
            from the bag file and the ground truth data. In the alignment, the
            estimator data will be shifted by this offset.
      - marker_calibration_file: Contains the calibration between the marker
            frame M and the eye frame I. If no file is given, the frames M and I
            are assumed to be identical.

    Returns:
        Sampled, aligned and transformed estimator trajectory, sampled ground
        truth.
    """
    if marker_calibration_file:
        estimator_data_unaligned_G_I = apply_hand_eye_calibration(
            estimator_data_unaligned_G_I, marker_calibration_file)
    else:
        estimator_data_unaligned_G_I = estimator_data_unaligned_G_I

    [estimator_data_G_I,
     ground_truth_data_W_M] = time_alignment.compute_aligned_poses(
         estimator_data_unaligned_G_I, ground_truth_data_unaligned_W_M,
         time_offset)

    # Align trajectories (umeyama).
    if align_data_to_use_meters < 0.0:
        umeyama_estimator_data_G_I = estimator_data_G_I[:, 1:4]
        umeyama_ground_truth_W_M = ground_truth_data_W_M[:, 1:4]
    else:
        trajectory_length = get_cumulative_trajectory_length_for_each_point(
            ground_truth_data_W_M[:, 1:4])
        align_num_data_points = bisect.bisect_left(trajectory_length,
                                                   align_data_to_use_meters)
        umeyama_estimator_data_G_I = estimator_data_G_I[:align_num_data_points,
                                                        1:4]
        umeyama_ground_truth_W_M = \
            ground_truth_data_W_M[:align_num_data_points, 1:4]

    T_W_G, scale = umeyama(umeyama_estimator_data_G_I.T,
                           umeyama_ground_truth_W_M.T, estimate_scale)
    assert T_W_G.shape[0] == 4
    assert T_W_G.shape[1] == 4

    if estimate_scale:
        assert scale > 0.0
        print("Estimator scale:", 1. / scale)

    estimator_data_G_I_scaled = estimator_data_G_I.copy()
    estimator_data_G_I_scaled[:, 1:4] *= scale

    ground_truth_data_G_M_aligned = ground_truth_data_W_M.copy()

    num_data_points = ground_truth_data_W_M.shape[0]
    for index in range(num_data_points):
        # Get transformation matrix from quaternion (normalized) and position.
        T_ground_truth_W_M = csv_row_data_to_transformation(
            ground_truth_data_W_M[index, 1:4],
            ground_truth_data_W_M[index, 4:8] / np.linalg.norm(
                ground_truth_data_W_M[index, 4:8]))
        # Apply found calibration between G and M frame.
        T_ground_truth_G_M = np.dot(np.linalg.inv(T_W_G), T_ground_truth_W_M)
        # Extract quaternion and position.
        q_ground_truth_G_M = Quaternion.from_rotation_matrix(
            T_ground_truth_G_M[0:3, 0:3])
        ground_truth_data_G_M_aligned[index, 4:8] = q_ground_truth_G_M.q
        ground_truth_data_G_M_aligned[index, 1:4] = T_ground_truth_G_M[0:3, 3]

    return estimator_data_G_I_scaled, ground_truth_data_G_M_aligned
