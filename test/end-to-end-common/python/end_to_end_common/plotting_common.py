#!/usr/bin/env python

import os

import numpy as np
import matplotlib.pyplot as plt


class PlottingSettings(object):
    def __init__(self,
                 show_plots=True,
                 save_path="",
                 save_format="pdf",
                 show_title=True,
                 show_grid=False):
        self.show_plots = show_plots
        self.save_path = save_path
        self.save_fig = len(save_path) > 0
        self.show_title = show_title
        self.show_grid = show_grid

        if self.save_fig:
            if not os.path.exists(self.save_path):
                os.makedirs(save_path)

        assert len(save_format) > 1
        if save_format[0] != ".":
            save_format = "." + save_format
        self.save_format = save_format

        # Settings for specific plots.
        self.trajectory_3d_axis_equal = False


def show_and_save_plot(plotting_settings, file_name):
    if plotting_settings.save_fig:
        plt.savefig(
            os.path.join(plotting_settings.save_path,
                         file_name + plotting_settings.save_format))

    if plotting_settings.show_plots:
        plt.show()


def axis_equal_3d(ax):
    extents = np.array(
        [getattr(ax, 'get_{}lim'.format(dim))() for dim in 'xyz'])
    sz = extents[:, 1] - extents[:, 0]
    centers = np.mean(extents, axis=1)
    max_size = max(abs(sz))
    r = max_size / 2
    for ctr, dim in zip(centers, 'xyz'):
        getattr(ax, 'set_{}lim'.format(dim))(ctr - r, ctr + r)


def plot_legend_on_top(ncol):
    """Creates a legend above the current plot.

   Inputs: ncol: number of columns, compare with matplotlib.pyplot.legend.

   Note when using with subplots: This will put the legend above the last made
   subplot, so it should be called after the first subplot has been added.
   """
    plt.legend(
        bbox_to_anchor=(0., 1.02, 1., .102),
        loc=3,
        ncol=ncol,
        mode="expand",
        borderaxespad=0.,
        frameon=False)
