import math
import numpy as np

from hand_eye_calibration.quaternion import Quaternion


def rmse(values):
    return math.sqrt(np.mean(np.square(values)))


class ErrorStatistics(object):
    def __init__(self, list_of_values):
        self.mean = np.mean(list_of_values)
        self.rmse = rmse(list_of_values)
        self.max_value = np.max(list_of_values)
        self.min_value = np.min(list_of_values)

    def __str__(self):
        return ('[mean = ' + str(self.mean) + ', RMSE = ' + str(self.rmse) +
                ', min = ' + str(self.min_value) + ', max = ' +
                str(self.max_value) + ']')


def compute_position_mean_and_rmse(p_A_list, p_B_list, axis=-1):
    """Computes the position mean and RMSE between two different point lists.

    If axis is a negative value, the mean and RMSE will be calculated from the
    norm of the difference between a pair of points in the two lists, otherwise
    only the specified axis is taken into account.
    """
    assert p_A_list.shape == p_B_list.shape
    assert p_A_list.shape[1] == 3
    assert axis < 3

    if axis < 0:
        errors_m = np.linalg.norm(p_A_list - p_B_list, axis=1)
    else:
        errors_m = np.abs(p_A_list[:, axis] - p_B_list[:, axis])

    return ErrorStatistics(errors_m)


def compute_orientation_mean_and_rmse_absolute(q_AB_list, q_AC_list):
    assert q_AB_list.shape == q_AC_list.shape
    assert q_AC_list.shape[1] == 4

    q_AB_0 = Quaternion(q_AB_list[0, 0], q_AB_list[0, 1], q_AB_list[0, 2],
                        q_AB_list[0, 3])
    q_AC_0 = Quaternion(q_AC_list[0, 0], q_AC_list[0, 1], q_AC_list[0, 2],
                        q_AC_list[0, 3])

    orientation_errors_rad = np.zeros(q_AB_list.shape[0])
    for idx in range(q_AB_list.shape[0]):
        q_AB_xyzw = q_AB_list[idx, :]
        q_AC_xyzw = q_AC_list[idx, :]
        q_AB = Quaternion(q_AB_xyzw[0], q_AB_xyzw[1], q_AB_xyzw[2],
                          q_AB_xyzw[3])
        q_AC = Quaternion(q_AC_xyzw[0], q_AC_xyzw[1], q_AC_xyzw[2],
                          q_AC_xyzw[3])

        # Zero the (arbitary) initial poses.
        q_AB = q_AB_0.inverse() * q_AB
        q_AC = q_AC_0.inverse() * q_AC

        q_BC = q_AB.inverse() * q_AC
        angle_BC = q_BC.angle_axis()[3]
        if angle_BC > np.pi:
            angle_BC -= 2 * np.pi

        assert q_AC_list.shape[1] == 4

        assert angle_BC <= np.pi
        assert angle_BC >= -np.pi
        orientation_errors_rad[idx] = np.abs(angle_BC)
    return ErrorStatistics(orientation_errors_rad)


def compute_orientation_mean_and_rmse_relative(q_AB_list, q_AC_list):
    assert q_AB_list.shape == q_AC_list.shape
    assert q_AC_list.shape[1] == 4

    orientation_errors_rad = np.zeros(q_AB_list.shape[0] - 1)
    for idx in range(q_AB_list.shape[0] - 1):
        q_ABk = Quaternion(q_AB_list[idx, 0], q_AB_list[idx, 1],
                           q_AB_list[idx, 2], q_AB_list[idx, 3])
        q_ACk = Quaternion(q_AC_list[idx, 0], q_AC_list[idx, 1],
                           q_AC_list[idx, 2], q_AC_list[idx, 3])

        q_ABkp1 = Quaternion(q_AB_list[idx + 1, 0], q_AB_list[idx + 1, 1],
                             q_AB_list[idx + 1, 2], q_AB_list[idx + 1, 3])
        q_ACkp1 = Quaternion(q_AC_list[idx + 1, 0], q_AC_list[idx + 1, 1],
                             q_AC_list[idx + 1, 2], q_AC_list[idx + 1, 3])

        q_Bk_Bkp1 = q_ABk.inverse() * q_ABkp1
        q_Ck_Ckp1 = q_ACk.inverse() * q_ACkp1

        q_error = q_Bk_Bkp1.inverse() * q_Ck_Ckp1
        angle_error = q_error.angle_axis()[3]

        if angle_error > np.pi:
            angle_error -= 2 * np.pi
        assert angle_error <= np.pi
        assert angle_error >= -np.pi
        orientation_errors_rad[idx] = np.abs(angle_error)
    return ErrorStatistics(orientation_errors_rad)


def compute_orientation_mean_and_rmse(q_AB_list,
                                      q_AC_list,
                                      use_relative_errors=False):
    if use_relative_errors:
        return compute_orientation_mean_and_rmse_relative(q_AB_list, q_AC_list)
    else:
        return compute_orientation_mean_and_rmse_absolute(q_AB_list, q_AC_list)
