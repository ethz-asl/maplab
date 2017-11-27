#!/usr/bin/env python

import numpy as np

# Force matplotlib to not use any Xwindows backend. Needs to be imported before
# any other matplotlib import (time_alignment import matplotlib).
import matplotlib
matplotlib.use('Agg')

import end_to_end_common.compute_rmse
from end_to_end_common.end_to_end_utility import align_datasets
import end_to_end_common.plot_rmse


class TestErrorStruct:
  def __init__(
      self, position_mean, position_rmse, orientation_mean, orientation_rmse):
    self.position_mean = position_mean
    self.position_rmse = position_rmse
    self.orientation_mean = orientation_mean
    self.orientation_rmse = orientation_rmse


class EndToEndTest:
  class TestResult:
    def __init__(
        self, label, calculated_errors, max_errors_list, estimator_G_I,
        ground_truth_G_M, localization_state_list):
      self.label = label
      self.calculated_errors = calculated_errors
      self.max_errors_list = max_errors_list
      self.ground_truth_G_M = ground_truth_G_M
      self.estimator_G_I = estimator_G_I
      self.localization_state_list = localization_state_list
      self.plot_result = False


  def __init__(self):
    self.test_results = []


  def calulate_errors_of_datasets(
          self, label, max_errors_list, estimated_poses_csv_path,
          ground_truth_csv_path, localization_state_list=np.zeros((0, 2)),
          estimator_csv_data_from_console=False):
    """
    Calculates the errors between the given datasets and appends the result to
    an internal list.

    Input:
      - label: name of the dataset that is compared. Used for later output and
            plotting.
      - max_errors_list: list containing TestErrorStruct which this dataset
            should test against.
      - estimated_poses_csv_path: path to the CSV containing the estimated poses
            from ROVIOLI or another estimator.
      - ground_truth_csv_path: path to the CSV containing the ground truth
            values for this dataset.
      - localization_state_list: (optional) Nx2-numpy-array where the first
            column contains timestamps and the second column localization
            results (1 = successful localization, 0 = no localization). The
            timestamps should match with the values in the estimator poses CSV
            file.
      - estimator_csv_data_from_console: (optional) if True, the estimator poses
            CSV file is assumed to be in the maplab console format, otherwise it
            will be assumed to be in the ROVIOLI output format.

    Returns:
      A TestErrorStruct struct containing the errors of the given dataset.
    """
    # TODO(eggerk): explain CSV formats somewhere.
    estimator_data_G_I, ground_truth_data_G_M = align_datasets(
        estimated_poses_csv_path, ground_truth_csv_path,
        estimator_csv_data_from_console)

    position_mean, position_rmse = \
        end_to_end_common.compute_rmse.compute_position_mean_and_rmse(
            estimator_data_G_I[:, 1:4], ground_truth_data_G_M[:, 1:4])
    orientation_mean, orientation_rmse = \
        end_to_end_common.compute_rmse.compute_orientation_mean_and_rmse(
            estimator_data_G_I[:, 4:8], ground_truth_data_G_M[:, 4:8])

    errors = TestErrorStruct(
        position_mean, position_rmse, orientation_mean, orientation_rmse)
    result = EndToEndTest.TestResult(
        label, errors, max_errors_list, estimator_data_G_I,
        ground_truth_data_G_M, localization_state_list)

    self.test_results.append(result)
    return errors


  def print_errors(self):
    """
    Prints all test errors (position and orientation mean and RMSE).
    """
    print "=" * 80
    for result in self.test_results:
      max_position_mean = 100000
      max_position_rmse = 100000
      max_orientation_mean = 100000
      max_orientation_rmse = 100000

      for max_errors in result.max_errors_list:
        max_position_mean = min(max_position_mean, max_errors.position_mean)
        max_position_rmse = min(max_position_rmse, max_errors.position_rmse)
        max_orientation_mean = min(
            max_orientation_mean, max_errors.orientation_mean)
        max_orientation_rmse = min(
            max_orientation_rmse, max_errors.orientation_rmse)

      print result.label, "errors:"
      print "    position mean [m]:      ", \
          result.calculated_errors.position_mean, \
          "\t(max:", max_position_mean, "\b)"
      print "    position RMSE [m]:      ", \
          result.calculated_errors.position_rmse, \
          "\t(max:", max_position_rmse, "\b)"
      print "    orientation mean [rad]: ", \
          result.calculated_errors.orientation_mean, \
          "\t(max:", max_orientation_mean, "\b)"
      print "    orientation RMSE [rad]: ", \
          result.calculated_errors.orientation_rmse, \
          "\t(max:", max_orientation_rmse, "\b)"
      print "=" * 80

  def check_errors(self):
    """
    Compares each of the stored test results if its values are below the given
    maximum test errors.
    """
    for result in self.test_results:
      print "Checking results from", result.label, "."
      assert result.calculated_errors.position_mean <= \
             result.calculated_errors.position_rmse
      assert result.calculated_errors.orientation_mean <= \
             result.calculated_errors.orientation_rmse
      for max_errors in result.max_errors_list:
        assert result.calculated_errors.position_mean < max_errors.position_mean
        assert result.calculated_errors.position_rmse < max_errors.position_rmse
        assert result.calculated_errors.orientation_mean < \
               max_errors.orientation_mean
        assert result.calculated_errors.orientation_rmse < \
               max_errors.orientation_rmse

      print "Done."


  def plot_results(self):
    """
    Creates a plot containing the position error for each given test dataset.
    """
    print "Plotting results."
    labels = []
    ground_truth_data = []
    estimator_data = []
    localizations = []
    for result in self.test_results:
      labels.append(result.label)
      ground_truth_data.append(result.ground_truth_G_M)
      estimator_data.append(result.estimator_G_I)
      localizations.append(result.localization_state_list)

    end_to_end_common.plot_rmse.plot_position_error(
        labels, ground_truth_data, estimator_data, localizations)
    print "Plotting done."
