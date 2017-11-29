import math
import numpy as np

from hand_eye_calibration.quaternion import Quaternion


def RMSE(values):
  return math.sqrt(np.mean(np.square(values)))


def compute_position_mean_and_rmse(p_A_list, p_B_list):
  assert(p_A_list.shape == p_B_list.shape)
  assert(p_A_list.shape[1] == 3)

  errors_m = np.linalg.norm(p_A_list - p_B_list, axis=1)

  return np.mean(errors_m), RMSE(errors_m)


def compute_orientation_mean_and_rmse(q_AB_list, q_AC_list):
  assert q_AB_list.shape == q_AC_list.shape
  assert(q_AC_list.shape[1] == 4)
  orientation_errors_rad = np.zeros(q_AB_list.shape[0])

  for idx in range(q_AB_list.shape[0]):
    q_AB_xyzw = q_AB_list[idx, :]
    q_AC_xyzw = q_AC_list[idx, :]
    q_AB = Quaternion(q_AB_xyzw[0], q_AB_xyzw[1], q_AB_xyzw[2], q_AB_xyzw[3])
    q_AC = Quaternion(q_AC_xyzw[0], q_AC_xyzw[1], q_AC_xyzw[2], q_AC_xyzw[3])
    q_BC = q_AB.inverse() * q_AC
    angle_BC = q_BC.angle_axis()[3]
    if angle_BC > np.pi:
      angle_BC -= 2 * np.pi
    orientation_errors_rad[idx] = angle_BC
  return np.mean(orientation_errors_rad), RMSE(orientation_errors_rad)
