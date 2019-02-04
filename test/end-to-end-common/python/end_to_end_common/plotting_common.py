#!/usr/bin/env python

import os

import matplotlib.pyplot as plt


class PlottingSettings:
  def __init__(
      self, show_plots=True, save_path="", save_format="pdf"):
    self.show_plots = show_plots
    self.save_path = save_path
    self.save_fig = len(save_path) > 0

    if self.save_fig:
      if not os.path.exists(self.save_path):
        os.makedirs(save_path)

    assert len(save_format) > 1
    if save_format[0] != ".":
      save_format = "." + save_format
    self.save_format = save_format


def show_and_save_plot(plotting_settings, file_name):
  if plotting_settings.save_fig:
    plt.savefig(
        os.path.join(
          plotting_settings.save_path,
          file_name + plotting_settings.save_format))

  if plotting_settings.show_plots:
    plt.show()
