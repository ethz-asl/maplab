#!/usr/bin/env python

import numpy as np

import end_to_end_common.compute_rmse
from end_to_end_common.end_to_end_utility import align_datasets
import end_to_end_common.plot_rmse


class TestErrorStruct:
  def __init__(
      self, position_mean_m, position_rmse_m, orientation_mean_rad,
      orientation_rmse_rad):
    self.position_mean_m = position_mean_m
    self.position_rmse_m = position_rmse_m
    self.orientation_mean_rad = orientation_mean_rad
    self.orientation_rmse_rad = orientation_rmse_rad


  def __str__(self):
    return str(self.position_mean_m) + ", " + \
        str(self.position_rmse_m) + ", " + str(self.orientation_mean_rad) + \
        ", " + str(self.orientation_rmse_rad)


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
      self.cpu_mean = -1
      self.cpu_stddev = -1


    def calculate_cpu_mean_and_stddev(self, cpu_file):
      if not os.path.isfile(cpu_file):
        return

      cpu_data = np.genfromtxt(cpu_file)
      self.cpu_mean = np.mean(cpu_data[:, 8])
      self.cpu_stddev = np.std(cpu_data[:, 8])


  def __init__(self):
    self.test_results = []


  def calulate_errors_of_datasets(
          self, label, max_errors_list, estimated_poses_csv_path,
          ground_truth_csv_path, localization_state_list=np.zeros((0, 2)),
          estimator_input_format="rovioli", cpu_file=""):
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
      - estimator_input_format: (optional) Determines the format of the given
            estimator data file.
            Possible values: rovioli, maplab_console, orbslam
            Default: rovioli
      - cpu_file: (optional) If provided, the end-to-end test will also compute
            the average CPU load for this dataset. Data is expected to be from
            top. Run top like this to get the data:
              ```bash
              # $1 = application name, $2 = output file
              while true; do
                echo "$(top -b -n 1 | grep $1)"  | tee -a $2
                sleep 1
              done
              ```
    Returns:
      A TestErrorStruct struct containing the errors of the given dataset.
    """
    # TODO(eggerk): explain CSV formats somewhere.
    estimator_data_G_I, ground_truth_data_G_M = align_datasets(
        estimated_poses_csv_path, ground_truth_csv_path, estimator_input_format)

    position_mean_m, position_rmse_m = \
        end_to_end_common.compute_rmse.compute_position_mean_and_rmse(
            estimator_data_G_I[:, 1:4], ground_truth_data_G_M[:, 1:4])
    orientation_mean_rad, orientation_rmse_rad = \
        end_to_end_common.compute_rmse.compute_orientation_mean_and_rmse(
            estimator_data_G_I[:, 4:8], ground_truth_data_G_M[:, 4:8])

    errors = TestErrorStruct(
        position_mean_m, position_rmse_m,
        orientation_mean_rad, orientation_rmse_rad)
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
        max_position_mean = min(max_position_mean, max_errors.position_mean_m)
        max_position_rmse = min(max_position_rmse, max_errors.position_rmse_m)
        max_orientation_mean = min(
            max_orientation_mean, max_errors.orientation_mean_rad)
        max_orientation_rmse = min(
            max_orientation_rmse, max_errors.orientation_rmse_rad)

      print result.label, "errors:"
      print "    position mean [m]:      ", \
          result.calculated_errors.position_mean_m, \
          "\t(max:", max_position_mean, "\b)"
      print "    position RMSE [m]:      ", \
          result.calculated_errors.position_rmse_m, \
          "\t(max:", max_position_rmse, "\b)"

      print "    orientation mean [deg]: ", \
          result.calculated_errors.orientation_mean_rad * 180 / np.pi, \
          "\t(max:", max_orientation_mean * 180 / np.pi, "\b)"
      print "    orientation RMSE [deg]: ", \
          result.calculated_errors.orientation_rmse_rad * 180 / np.pi, \
          "\t(max:", max_orientation_rmse * 180 / np.pi, "\b)"

      if result.cpu_mean > 0 and result.cpu_stddev >= 0:
        print "    CPU load [%]:\tmean:", result.cpu_mean, "\tstd dev:", \
            result.cpu_stddev

      print "=" * 80

  def check_errors(self):
    """
    Compares each of the stored test results if its values are below the given
    maximum test errors.
    """
    for result in self.test_results:
      print "Checking results from", result.label, "."
      assert result.calculated_errors.position_mean_m <= \
             result.calculated_errors.position_rmse_m
      assert result.calculated_errors.orientation_mean_rad <= \
             result.calculated_errors.orientation_rmse_rad
      for max_errors in result.max_errors_list:
        assert result.calculated_errors.position_mean_m < \
            max_errors.position_mean_m
        assert result.calculated_errors.position_rmse_m < \
            max_errors.position_rmse_m
        assert result.calculated_errors.orientation_mean_rad < \
               max_errors.orientation_mean_rad
        assert result.calculated_errors.orientation_rmse_rad < \
               max_errors.orientation_rmse_rad

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
