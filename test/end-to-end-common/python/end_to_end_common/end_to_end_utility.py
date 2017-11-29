#!/usr/bin/env python

import math
import os

import numpy as np

from hand_eye_calibration.quaternion import Quaternion
import hand_eye_calibration.time_alignment as time_alignment
from end_to_end_common.umeyama import umeyama


def csv_row_data_to_transformation(A_p_AB, q_AB_np):
  """
  Constructs a proper transformation matrix out of the given translation vector
  and quaternion.

  Returns: 4x4 transformation matrix.
  """
  assert(math.fabs(1.0 - np.linalg.norm(q_AB_np)) < 1e-8)
  T_AB = np.identity(4)
  q_AB = Quaternion(q=q_AB_np)
  R_AB = q_AB.to_rotation_matrix()
  T_AB[0:3, 0:3] = R_AB
  T_AB[0:3, 3] = A_p_AB
  return T_AB


def convert_ground_truth_data_into_target_format(ground_truth_data):
  # We need to bring the Euroc ground truth data into the format required
  # by hand-eye calibration.
  # Convert timestamp ns -> s.
  ground_truth_data[:, 0] *= 1e-9
  # Reorder quaternion wxyz -> xyzw.
  ground_truth_data[:, [4, 5, 6, 7]] = ground_truth_data[:, [5, 6, 7, 4]]
  # Remove remaining columns.
  ground_truth_data = ground_truth_data[:, 0:8]
  return ground_truth_data


def align_datasets(
        estimator_data_G_I_path,
        ground_truth_data_W_M_path,
        estimator_input_format="rovioli"):
  """
  Sample and align the ground truth trajectory to the estimator trajectory.

  Input frames:
      Estimator:      G (Global, from rovioli) --> I (imu, from rovioli)

      Ground truth:   W (Ground truth world, e.g., Vicon frame)
                       --> M (ground truth marker, e.g., Vicon marker)
      (M and I should be at the same position at a given time.)

  Returns: aligned estimator trajectory, aligned and sampled ground truth
  trajectory.
  Output data frames:
      Estimator:      G --> I

      Ground truth:   G --> M
      (M and I should be at the same position.)
  """
  assert os.path.isfile(estimator_data_G_I_path)
  assert os.path.isfile(ground_truth_data_W_M_path)

  estimator_data_unaligned_G_I = np.zeros((0, 8))
  if estimator_input_format.lower() == "maplab_console":
    estimator_data_unaligned_G_I = np.genfromtxt(
        estimator_data_G_I_path, delimiter=',', skip_header=1)
    estimator_data_unaligned_G_I = estimator_data_unaligned_G_I[:, 1:9]
    # Convert timestamp ns -> s.
    estimator_data_unaligned_G_I[:, 0] *= 1e-9
  elif estimator_input_format.lower() == "rovioli":
    # Compute T_G_I from T_G_M and T_G_I
    estimator_data_raw = np.genfromtxt(
        estimator_data_G_I_path, delimiter=',', skip_header=1)
    num_elements = estimator_data_raw.shape[0]
    estimator_data_unaligned_G_I = np.zeros((num_elements, 8))
    for i in range(num_elements):
      estimator_data_unaligned_G_I[i, 0] = estimator_data_raw[i, 0]

      T_G_M = csv_row_data_to_transformation(
          estimator_data_raw[i, 1:4], estimator_data_raw[i, 4:8])
      T_M_I = csv_row_data_to_transformation(
          estimator_data_raw[i, 8:11], estimator_data_raw[i, 11:15])
      T_G_M = np.dot(T_G_M, T_M_I)
      estimator_data_unaligned_G_I[i, 1:4] = T_G_M[0:3, 3]
      q_G_I = Quaternion.from_rotation_matrix(T_G_M[0:3, 0:3])
      estimator_data_unaligned_G_I[i, 4:8] = q_G_I.q
  elif estimator_input_format.lower() == "orbslam":
    estimator_data_unaligned_G_I = np.genfromtxt(
        estimator_data_G_I_path, delimiter=' ', skip_header=0)
    # TODO(eggerk): Check quaternion convention from ORBSLAM.
  else:
    print "Unknown estimator input format."
    print "Valid options are maplab_console, rovioli, orbslam."
    assert False

  ground_truth_data_unaligned_W_M = np.genfromtxt(
      ground_truth_data_W_M_path, delimiter=',', skip_header=1)
  ground_truth_data_unaligned_W_M = \
      convert_ground_truth_data_into_target_format(
          ground_truth_data_unaligned_W_M)

  # This is needed for the alignment and sampling. It indicates the time offset
  # between the data from Rovioli/from the bag file and the ground truth data.
  # In the case of the Euroc datasets, this is 0 as they image/imu data and
  # ground truth is already synchronized.
  time_offset = 0
  [ground_truth_data_W_M, estimator_data_G_I] = \
      time_alignment.compute_aligned_poses(
          ground_truth_data_unaligned_W_M, estimator_data_unaligned_G_I,
          time_offset)

  # Create a matrix of the right size to store aligned trajectory.
  ground_truth_data_aligned_G_M = ground_truth_data_W_M

  # Align trajectories (umeyama).
  R_G_W, t_G_W, scale = umeyama(
      ground_truth_data_W_M[:, 1:4].T, estimator_data_G_I[:, 1:4].T)
  T_G_W = np.identity(4)
  T_G_W[0:3, 0:3] = R_G_W
  T_G_W[0:3, 3] = t_G_W
  if abs(1 - scale) < 1e-8:
    print "WARNING: Scale is not 1, but", scale, "\b."

  num_data_points = estimator_data_G_I.shape[0]
  for index in range(num_data_points):
    # Create transformations to compare.
    T_ground_truth_G_M = np.dot(T_G_W, csv_row_data_to_transformation(
        ground_truth_data_W_M[index, 1:4],
        ground_truth_data_W_M[index, 4:8] /
            np.linalg.norm(ground_truth_data_W_M[index, 4:8])))
    ground_truth_data_aligned_G_M[index, 1:4] = T_ground_truth_G_M[0:3, 3]
    q_ground_truth_G_M = Quaternion.from_rotation_matrix(
        T_ground_truth_G_M[0:3, 0:3])
    ground_truth_data_aligned_G_M[index, 4:8] = q_ground_truth_G_M.q

  return estimator_data_G_I, ground_truth_data_aligned_G_M
