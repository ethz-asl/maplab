#!/usr/bin/env python

from __future__ import print_function

import argparse
import copy
import os
import sys

import numpy as np

import end_to_end_common.compute_rmse
import end_to_end_common.create_plots
from end_to_end_common.end_to_end_utility import align_datasets
from end_to_end_common.test_structs import TestDataStruct, TestErrorStruct


def rad_to_deg(rad):
    return rad * 180 / np.pi


class EndToEndTest(object):
    def __init__(self):
        self.test_results = {}

    def calculate_errors_of_datasets(self,
                                     label,
                                     estimated_poses_csv_G_I,
                                     ground_truth_csv_W_M,
                                     estimate_scale=False,
                                     align_data_to_use_meters=-1,
                                     time_offset=0.0,
                                     max_errors_list=None,
                                     localization_state_list=np.zeros((0, 2))):
        """Calculates the errors between the given datasets and appends the
        result to an internal list.

        Input:
          - label: name of the dataset that is compared. Used for later output
                and plotting.
          - estimated_poses_csv_G_I: path to the CSV containing the estimated
                poses from ROVIOLI or another estimator.
          - ground_truth_csv_W_M: path to the CSV containing the ground truth
                values for this dataset.
          - estimate_scale: Estimate a scale factor in addition to the transform
                and apply it to the aligned estimator trajectory.
          - align_data_to_use_meters: (optional) only use data up to a total
                trajectory length as given by this parameter. If a negative
                value is provided, the entire trajectory is used for the
                alignment.
          - time_offset: (optional) Time offset in seconds by which the
                estimator time stamps will be shifted in the alignment.
          - max_errors_list: list containing TestErrorStruct which this dataset
                should test against.
          - localization_state_list: (optional) Nx2-numpy-array where the first
                column contains timestamps and the second column localization
                results (1 = successful localization, 0 = no localization). The
                timestamps should match with the values in the estimator poses
                CSV file.
        Returns:
          A TestErrorStruct struct containing the errors of the given dataset.
        """
        # TODO(eggerk): explain CSV formats somewhere.
        assert label not in self.test_results, \
            "A dataset with the label " + label + " was already run."
        if max_errors_list is None:
            max_errors_list = []

        estimator_data_W_I, ground_truth_data_W_M = align_datasets(
            estimated_poses_csv_G_I,
            ground_truth_csv_W_M,
            estimate_scale,
            align_data_to_use_meters=align_data_to_use_meters,
            time_offset=time_offset)

        position_errors_m = \
            end_to_end_common.compute_rmse.compute_position_mean_and_rmse(
                estimator_data_W_I[:, 1:4], ground_truth_data_W_M[:, 1:4])
        orientation_errors_rad = \
            end_to_end_common.compute_rmse.compute_orientation_mean_and_rmse(
                estimator_data_W_I[:, 4:8], ground_truth_data_W_M[:, 4:8])

        errors = TestErrorStruct()
        errors.set_position_errors(position_errors_m)
        errors.set_orientation_errors(orientation_errors_rad)
        result = TestDataStruct(
            estimator_data_W_I,
            ground_truth_data_W_M,
            label=label,
            calculated_errors=errors,
            max_errors_list=max_errors_list,
            localization_state_list=localization_state_list)

        self.test_results.update({label: result})
        return copy.copy(errors)

    def print_errors(self):
        """Prints all test errors (position and orientation mean and RMSE)."""
        for label, result in self.test_results.iteritems():
            print(label, "errors:")
            print("    position [m]:\tmean =",
                  result.calculated_errors.position_mean_m, "\tRMSE =",
                  result.calculated_errors.position_rmse_m, "\tmin =",
                  result.calculated_errors.position_min_error_m, "\tmax =",
                  result.calculated_errors.position_max_error_m)
            if result.max_errors.position_mean_m >= 0 or \
                result.max_errors.position_rmse_m >= 0:
                print(
                    "        (Thresholds:\tmean = ",
                    result.max_errors.position_mean_m,
                    "\tRMSE = ",
                    result.max_errors.position_rmse_m,
                    ")",
                    sep="")

            print("    orientation [deg]:\tmean =", rad_to_deg(
                result.calculated_errors.orientation_mean_rad), "\tRMSE =",
                  rad_to_deg(result.calculated_errors.orientation_rmse_rad),
                  "\tmin =", rad_to_deg(
                      result.calculated_errors.orientation_min_error_rad),
                  "\tmax =", rad_to_deg(
                      result.calculated_errors.orientation_max_error_rad))
            if result.max_errors.orientation_mean_rad >= 0 or \
                result.max_errors.orientation_rmse_rad >= 0:
                print(
                    "        (Thresholds:\tmean = ",
                    rad_to_deg(result.max_errors.orientation_mean_rad),
                    "\tRMSE = ",
                    rad_to_deg(result.max_errors.orientation_rmse_rad),
                    ")",
                    sep="")

            if result.cpu_mean > 0 and result.cpu_stddev >= 0:
                print("    CPU load [%]:\tmean:", result.cpu_mean,
                      "\tstd dev:", result.cpu_stddev)

            print("=" * 80)

    def check_errors(self):
        """Compares each of the stored test results if its values are below the
        given maximum test errors.
        """
        for label, result in self.test_results.iteritems():
            print("Checking results from ", label, ".", sep="")
            result.check_errors()

        print("Done.")

    def get_output_file_name(self, export_path, label):
        if os.path.isfile(label):
            file_name = os.path.basename(label)
        else:
            file_name = label.replace("/", "_")
        return os.path.join(export_path, file_name + "_errors.yaml")

    def export_results_to_file(self, export_path):
        assert export_path
        for label, _ in self.test_results.iteritems():
            path = self.get_output_file_name(export_path, label)
            self.export_results_to_file_for_label(path, label)

    def export_results_to_file_for_label(self, export_file_name, label):
        if not os.path.isdir(os.path.dirname(export_file_name)):
            os.makedirs(os.path.dirname(export_file_name))
        errors = self.test_results[label].calculated_errors
        error_thresholds = self.test_results[label].max_errors
        out_file = open(export_file_name, "w+")
        out_file.write("metadata:\n")
        out_file.write("  label: %s\n" % label)
        out_file.write("position_errors:  # [m]\n")
        out_file.write("  mean: %f\n" % errors.position_mean_m)
        out_file.write("  rmse: %f\n" % errors.position_rmse_m)
        out_file.write("  min: %f\n" % errors.position_min_error_m)
        out_file.write("  max: %f\n" % errors.position_max_error_m)
        if error_thresholds.position_mean_m > 0 or \
            error_thresholds.position_rmse_m > 0:
            out_file.write("  thresholds:\n")
            if error_thresholds.position_mean_m > 0:
                out_file.write(
                    "    mean: %f\n" % error_thresholds.position_mean_m)
            if error_thresholds.position_rmse_m > 0:
                out_file.write(
                    "    rmse: %f\n" % error_thresholds.position_rmse_m)
        out_file.write("orientation_errors:  # [rad]\n")
        out_file.write("  mean: %f\n" % errors.orientation_mean_rad)
        out_file.write("  rmse: %f\n" % errors.orientation_rmse_rad)
        out_file.write("  min: %f\n" % errors.orientation_min_error_rad)
        out_file.write("  max: %f\n" % errors.orientation_max_error_rad)
        if error_thresholds.orientation_mean_rad > 0 or \
            error_thresholds.orientation_rmse_rad > 0:
            out_file.write("  thresholds:\n")
            if error_thresholds.orientation_mean_rad > 0:
                out_file.write(
                    "    mean: %f\n" % error_thresholds.orientation_mean_rad)
            if error_thresholds.orientation_rmse_rad > 0:
                out_file.write(
                    "    rmse: %f\n" % error_thresholds.orientation_rmse_rad)
        out_file.close()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="End to end test")
    parser.add_argument(
        "--max_position_rmse_m",
        dest="max_position_rmse_m",
        type=float,
        required=False,
        default=10)
    parser.add_argument(
        "--max_orientation_rmse_rad",
        dest="max_orientation_rmse_rad",
        type=float,
        required=False,
        default=10)
    parser.add_argument(
        "--ground_truth_csv_file", dest="ground_truth_csv_file", required=True)
    parser.add_argument(
        "--estimator_csv_file", dest="estimator_csv_file", required=True)
    parser.add_argument(
        "--estimator_csv_format",
        dest="estimator_csv_format",
        required=False,
        default="rovioli")
    parser.add_argument(
        "--label", dest="label", required=False, default="Dataset")
    parser.add_argument(
        "--min_position_rmse_m",
        dest="min_position_rmse_m",
        type=float,
        required=False,
        default=-1)
    parser.add_argument(
        "--save_results_to_file",
        dest="save_results_to_file",
        required=False,
        default="",
        help="Optionally specify a path where to save the test results.")

    parsed_args = parser.parse_args()

    error_struct = TestErrorStruct(parsed_args.max_position_rmse_m,
                                   parsed_args.max_position_rmse_m,
                                   parsed_args.max_orientation_rmse_rad,
                                   parsed_args.max_orientation_rmse_rad)
    test = EndToEndTest()
    # TODO(eggerk): update to new function.
    test_results = test.calculate_errors_of_datasets(
        parsed_args.label, [error_struct], parsed_args.estimator_csv_file,
        parsed_args.ground_truth_csv_file, False)
    test.print_errors()
    exit_code = 0
    try:
        test.check_errors()
    except:  # pylint: disable=bare-except
        print("The RMSE of the dataset is worse than what is allowed!")
        exit_code += 1

    if test_results.position_rmse_m < parsed_args.min_position_rmse_m:
        print("The RMSE of ", test_results.position_rmse_m, \
              "m is below the expected minimum (which is ", \
              parsed_args.min_position_rmse_m, ")", sep="")
        exit_code += 2

    if parsed_args.save_results_to_file:
        test.export_results_to_file(parsed_args.save_results_to_file)

    sys.exit(exit_code)
