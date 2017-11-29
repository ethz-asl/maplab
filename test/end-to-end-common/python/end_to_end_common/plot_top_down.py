#!/usr/bin/env python

import sys

import numpy as np
import matplotlib.pyplot as plt

from end_to_end_common.end_to_end_utility import align_datasets


def plot_top_down_unaligned(
        estimator_data_csv_path, ground_truth_csv_path, input_format="rovioli"):
  """
  Creates a top-down plot of the given estimated and ground truth trajectory.

  Inputs: path to the CSV file containing the (unaligned) estimator and ground
  truth trajectory.
  """
  estimator_data_G_I, ground_truth_data_G_M = align_datasets(
      estimator_data_csv_path, ground_truth_csv_path,
      estimator_input_format=input_format)
  plot_top_down_aligned_data(estimator_data_G_I, ground_truth_data_G_M)


def plot_top_down_aligned_data(estimator_data_G_I, ground_truth_data_G_M):
  """
  Creates a top-down plot of the given estimated and ground truth trajectory.

  Inputs: aligned estimator and ground truth trajectory matrices (assumed to be
  in the output format of end_to_end_common.end_to_end_utility.align_datasets).
  """
  plt.plot(
      ground_truth_data_G_M[:, 1], ground_truth_data_G_M[:, 2],
      label="Ground truth")
  plt.plot(
      estimator_data_G_I[:, 1], estimator_data_G_I[:, 2], label="Estimator")
  plt.xlabel("x position [m]")
  plt.ylabel("y position [m]")
  plt.legend()
  plt.title(estimator_data_csv_path)
  plt.show()


if __name__ == "__main__":
  if len(sys.argv) < 3:
    print "Please provide estimator CSV file path and ground truth CSV file path."
    sys.exit(1)

  estimator_data_csv_path = sys.argv[1]
  ground_truth_csv_path = sys.argv[2]
  input_format = "rovioli"
  if len(sys.argv) > 3:
    input_format = sys.argv[3]

  plot_top_down_unaligned(
      estimator_data_csv_path, ground_truth_csv_path, input_format)
