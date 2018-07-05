#!/usr/bin/env python

import os

from evaluation_tools.catkin_utils import catkinFindSrc

import maplab_evaluation.perform_evaluation as evaluation


def test_ordering():
    path_to_search = os.path.join(
        catkinFindSrc('maplab_common'), 'include', 'maplab-common')
    file_filter = 'file*.h'
    files = evaluation.get_filtered_files_in_folder_ordered(
        path_to_search, file_filter)
    assert len(files) >= 2, \
        'There were less than two files found in the search folder ("' + \
        path_to_search + '", filter "' + file_filter + \
        '"), test only makes sense if more than two files are found.'

    for idx in range(len(files) - 1):
        first_file = files[idx]
        second_file = files[idx + 1]
        assert first_file <= second_file, 'Wrong order: "' + first_file + \
            '" is before "' + second_file + '".'
