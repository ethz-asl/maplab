#!/usr/bin/env python

import glob
import itertools
import os

from maplab_evaluation.report_common import generate_random_string, sanitize


class ReportGeneratorBase(object):
    def __init__(self):
        self.job_path_to_id = {}
        self.all_jobs_sorted = []

    def _generate_report(self, output_folder, latex_main_file):
        self._write_summary()
        last_job_prefix = ''
        for job in self.all_jobs_sorted:
            last_job_prefix = self._write_report_for_job(job, last_job_prefix)

        self._write_base_structure()

        # Do it twice for references.
        for _ in range(2):
            os.system(
                'pdflatex -interaction=nonstopmode --output-directory %s %s' %
                (output_folder, os.path.realpath(latex_main_file)))

    def _write_summary(self):
        pass

    def _write_report_for_job(self, job, last_job_prefix):
        pass

    def _write_base_structure(self):
        pass

    def _create_random_ids_for_all_jobs(self, all_jobs):
        self.job_path_to_id = dict(
            (job, generate_random_string(10)) for job in all_jobs)
        self.all_jobs_sorted = sorted(all_jobs)

    def _experiment_name_to_title(self, name):
        name_split = name.split('_')
        if len(name_split) <= 2:
            return name
        if len(name_split[0]) != 8 and len(name_split[1]) != 6:
            # Not a correct timestamp.
            return name
        title = '\\texttt{' + sanitize('_'.join(name_split[2:])) + '} ('
        title += (sanitize(name_split[0][6:8]) + '.' + sanitize(
            name_split[0][4:6]) + '.' + sanitize(name_split[0][0:4]))
        title += (', ' + sanitize(name_split[1][0:2]) + ':' + sanitize(
            name_split[1][2:4]) + ':' + sanitize(name_split[1][4:6])) + ')'
        return title

    def _job_name_to_title(self, name):
        name_split = name.split('__')
        if len(name_split) != 2:
            return name
        return ('Dataset \\texttt{' + sanitize(name_split[0]) +
                '}, parameters \\texttt{' + sanitize(name_split[1]) + '}')

    def _get_section_header_string_for_element(self, job_base_name,
                                               path_of_file, title, last_path):
        index_after_which_to_use = path_of_file.index(job_base_name) + 1
        shortened_path = path_of_file[index_after_which_to_use:]
        if not shortened_path:
            level = '\\subsubsection'
        elif len(shortened_path) == 1:
            level = '\\myparagraph'
        elif len(shortened_path) == 2:
            level = '\\mysubparagraph'
        elif len(shortened_path) == 3:
            level = '\\mysubsubparagraph'
        output = ''
        for idx, path_element in enumerate(shortened_path):
            if len(last_path) <= idx or path_element != last_path[idx]:
                if idx == 0:
                    output += '\n\\subsubsection{%s}\n' % sanitize(
                        path_element)
                    level = '\\myparagraph'
                elif idx == 1:
                    output += '\n\\myparagraph{%s}\n' % sanitize(path_element)
                    level = '\\mysubparagraph'
                elif idx == 2:
                    output += '\n\\mysubparagraph{%s}\n' % sanitize(
                        path_element)
                    level = '\\mysubsubparagraph'
        last_path = shortened_path
        if len(shortened_path) > 3:
            path_to_print = os.path.join('/'.join(shortened_path[3:]), title)
        else:
            path_to_print = title
        output += '\n%s{%s}\n' % (level, sanitize(path_to_print))
        return output, last_path

    def _get_all_files_filtered_in_job(self, job_path, file_filter):
        return list(
            itertools.chain.from_iterable(
                glob.iglob(os.path.join(root, file_filter))
                for root, dirs, files in os.walk(job_path)))
