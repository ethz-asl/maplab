#!/usr/bin/env python

import math
import os

import numpy as np

from hand_eye_calibration.quaternion import Quaternion


def csv_row_data_to_transformation(A_p_AB, q_AB_np):
    """
    Constructs a proper transformation matrix out of the given translation
    vector and quaternion (xyzw).

    Returns: 4x4 transformation matrix.
    """
    assert math.fabs(1.0 - np.linalg.norm(q_AB_np)) < 1e-8
    T_AB = np.identity(4)
    q_AB = Quaternion(q=q_AB_np)
    R_AB = q_AB.to_rotation_matrix()
    T_AB[0:3, 0:3] = R_AB
    T_AB[0:3, 3] = A_p_AB
    return T_AB


def load_dataset(csv_file, input_format="maplab_console"):
    """
    Loads a dataset in one of the supported formats and returns a Nx8 matrix
    with the trajectory from the dataset with the following columns:
    timestamp [s], position x, y, z [m], quaternion x, y, z, w

    Supported formats:
      - end_to_end (same as internal end_to_end representation)
      - maplab_console (output of csv_export command in maplab)
      - rovioli (output of --export_estimated_poses_to_csv flag from ROVIOLI)
      - euroc_ground_truth (euroc ground truth CSVs as can be downloaded with
                            the datasets)
      - orbslam (output from running ORB-SLAM)
    """
    assert os.path.isfile(csv_file), "File " + csv_file + " doesn't exist."
    NANOSECONDS_TO_SECONDS = 1e-9

    trajectory = np.zeros((0, 8))
    if input_format.lower() == "end_to_end":
        trajectory = np.genfromtxt(csv_file, delimiter=',', skip_header=0)
    elif input_format.lower() == "maplab_console":
        trajectory = np.genfromtxt(csv_file, delimiter=',', skip_header=1)
        trajectory = trajectory[:, 1:9]
        trajectory[:, 0] *= NANOSECONDS_TO_SECONDS
    elif input_format.lower() == "rovioli":
        # Compute T_G_I from T_G_M and T_G_I.
        estimator_data_raw = np.genfromtxt(
            csv_file, delimiter=',', skip_header=1)
        num_elements = estimator_data_raw.shape[0]
        trajectory = np.zeros((num_elements, 8))
        for i in range(num_elements):
            trajectory[i, 0] = estimator_data_raw[i, 0]

            T_G_M = csv_row_data_to_transformation(estimator_data_raw[i, 1:4],
                                                   estimator_data_raw[i, 4:8])
            T_M_I = csv_row_data_to_transformation(
                estimator_data_raw[i, 8:11], estimator_data_raw[i, 11:15])
            T_G_M = np.dot(T_G_M, T_M_I)
            trajectory[i, 1:4] = T_G_M[0:3, 3]
            q_G_I = Quaternion.from_rotation_matrix(T_G_M[0:3, 0:3])
            trajectory[i, 4:8] = q_G_I.q
    elif input_format.lower() == "euroc_ground_truth":
        trajectory = np.genfromtxt(csv_file, delimiter=',', skip_header=1)
        # We need to bring the Euroc ground truth data into the format required
        # by hand-eye calibration.
        trajectory[:, 0] *= NANOSECONDS_TO_SECONDS
        # Reorder quaternion wxyz -> xyzw.
        trajectory[:, [4, 5, 6, 7]] = trajectory[:, [5, 6, 7, 4]]
        # Remove remaining columns.
        trajectory = trajectory[:, 0:8]
    elif input_format.lower() == "orbslam":
        trajectory = np.genfromtxt(csv_file, delimiter=' ', skip_header=0)
        # TODO(eggerk): Check quaternion convention from ORBSLAM.
    else:
        print "Unknown estimator input format."
        print "Valid options are maplab_console, rovioli, orbslam."
        assert False

    return trajectory
