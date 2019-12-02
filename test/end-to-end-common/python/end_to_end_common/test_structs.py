#!/usr/bin/env python

import os

import numpy as np


class TestErrorStruct(object):
    def __init__(self,
                 position_mean_m=-1,
                 position_rmse_m=-1,
                 orientation_mean_rad=-1,
                 orientation_rmse_rad=-1):
        self.position_mean_m = position_mean_m
        self.position_rmse_m = position_rmse_m
        self.position_max_error_m = -1
        self.position_min_error_m = -1
        self.orientation_mean_rad = orientation_mean_rad
        self.orientation_rmse_rad = orientation_rmse_rad
        self.orientation_max_error_rad = -1
        self.orientation_min_error_rad = -1
        self.axis_position_errors = []

    def set_position_errors(self, position_errors):
        self.position_mean_m = position_errors.mean
        self.position_rmse_m = position_errors.rmse
        self.position_max_error_m = position_errors.max_value
        self.position_min_error_m = position_errors.min_value

    def set_orientation_errors(self, orientation_errors):
        self.orientation_mean_rad = orientation_errors.mean
        self.orientation_rmse_rad = orientation_errors.rmse
        self.orientation_max_error_rad = orientation_errors.max_value
        self.orientation_min_error_rad = orientation_errors.min_value

    def __str__(self):
        return str(self.position_mean_m) + ", " + str(
            self.position_rmse_m) + ", " + str(
                self.orientation_mean_rad) + ", " + str(
                    self.orientation_rmse_rad)


class ThresholdExceededException(Exception):
    pass


class TestDataStruct(object):
    def __init__(self,
                 estimator_G_I,
                 ground_truth_G_M,
                 label="",
                 calculated_errors=TestErrorStruct(),
                 max_errors_list=None,
                 localization_state_list=np.zeros((0, 2))):
        if max_errors_list is None:
            max_errors_list = []

        self.label = label
        self.calculated_errors = calculated_errors

        self.max_errors = TestErrorStruct(-1, -1, -1, -1)
        for max_errors in max_errors_list:
            self.max_errors.position_mean_m = self._get_min_positive_value(
                self.max_errors.position_mean_m, max_errors.position_mean_m)
            self.max_errors.position_rmse_m = self._get_min_positive_value(
                self.max_errors.position_rmse_m, max_errors.position_rmse_m)
            self.max_errors.orientation_mean_rad = self._get_min_positive_value(
                self.max_errors.orientation_mean_rad,
                max_errors.orientation_mean_rad)
            self.max_errors.orientation_rmse_rad = self._get_min_positive_value(
                self.max_errors.orientation_rmse_rad,
                max_errors.orientation_rmse_rad)

        self.ground_truth_G_M = ground_truth_G_M
        self.estimator_G_I = estimator_G_I
        self.localization_state_list = localization_state_list
        self.plot_result = False
        self.cpu_mean = -1
        self.cpu_stddev = -1

    def _get_min_positive_value(self, a, b):
        """Returns the minimum value of a and b ignoring any negative values.

        The following cases are differentiated:
        - if a < 0 and b >= 0: return b
        - if a >= 0 and b < 0: return a
        - otherwise: return min(a, b)
        """
        if a < 0 and b >= 0:
            return b
        if a >= 0 and b < 0:
            return a
        return min(a, b)

    def calculate_cpu_mean_and_stddev(self, cpu_file):
        if not os.path.isfile(cpu_file):
            return

        cpu_data = np.genfromtxt(cpu_file)
        self.cpu_mean = np.mean(cpu_data[:, 8])
        self.cpu_stddev = np.std(cpu_data[:, 8])

    def _check_value(self, text, calculated_error, max_error):
        if max_error > 0:
            if calculated_error > max_error:
                raise ThresholdExceededException(
                    "The %s (%f) for the dataset %s exceeds the maximum "
                    "allowed value (%f)." % (text, calculated_error,
                                             self.label, max_error))

    def check_errors(self):
        assert (self.calculated_errors.position_mean_m <=
                self.calculated_errors.position_rmse_m)
        assert (self.calculated_errors.orientation_mean_rad <=
                self.calculated_errors.orientation_rmse_rad)

        if (self.calculated_errors.position_mean_m >= 0
                and self.calculated_errors.position_rmse_m >= 0):
            self._check_value("position mean",
                              self.calculated_errors.position_mean_m,
                              self.max_errors.position_mean_m)
            self._check_value("position RMSE",
                              self.calculated_errors.position_rmse_m,
                              self.max_errors.position_rmse_m)

        if (self.calculated_errors.orientation_mean_rad >= 0
                and self.calculated_errors.orientation_rmse_rad >= 0):
            self._check_value("orientation mean",
                              self.calculated_errors.orientation_mean_rad,
                              self.max_errors.orientation_mean_rad)
            self._check_value("orientation RMSE",
                              self.calculated_errors.orientation_rmse_rad,
                              self.max_errors.orientation_rmse_rad)
