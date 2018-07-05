#!/usr/bin/env python

from __future__ import print_function

import os

from evaluation_tools.catkin_utils import catkinFindSrc

from maplab_evaluation.report_common import sanitize
from maplab_evaluation.report_generator_base import ReportGeneratorBase
import maplab_evaluation.yaml_outputter as yaml_outputter


class ReportGenerator(ReportGeneratorBase):
    def __init__(self, results_folder, evaluation_results):
        ReportGeneratorBase.__init__(self)
        print("Writing Latex report.")
        self.results_folder = results_folder
        self.report_file = os.path.join(results_folder, 'report.tex')
        self.evaluation_results = evaluation_results

        self._create_random_ids_for_all_jobs(
            [job for job in self.evaluation_results.iterkeys()])

    def generate_report(self):
        self._generate_report(
            output_folder=self.results_folder,
            latex_main_file=self.report_file)

    def _write_base_structure(self):
        template_file_path = os.path.join(
            catkinFindSrc('maplab_evaluation'), 'latex', 'template.tex')
        with open(template_file_path, 'r') as template_file:
            template = template_file.read()

        content_summary = '\\input{%s}\n' % os.path.realpath(
            os.path.join(self.results_folder, 'report_summary.tex'))

        last_job_prefix = None
        content_all_jobs = ''
        for job_path in self.all_jobs_sorted:
            job_prefix = os.path.dirname(job_path)
            if job_prefix != last_job_prefix:
                last_job_prefix = job_prefix
                content_all_jobs += '\\pagebreak\n'
                content_all_jobs += '\n\\section{%s}\n' % (
                    self._experiment_name_to_title(job_prefix))
            job_report = os.path.realpath(
                os.path.join(self.results_folder, job_path, 'job_report.tex'))
            if os.path.isfile(job_report):
                content_all_jobs += '\\input{%s}\n' % job_report

        with open(self.report_file, 'w+') as output_file:
            output_file.write(
                template.replace('SUMMARYCONTENT', content_summary).replace(
                    'PERJOBCONTENT', content_all_jobs))

    def _write_summary(self):
        with open(
                os.path.join(self.results_folder, 'report_summary.tex'),
                'w+') as output_file:
            output_file.write('\\section*{Summary}\n')
            output_file.write('\\addcontentsline{toc}{section}{Summary}')
            output_file.write(
                '\\begin{longtabu} to \\textwidth {lX[l]X[l]|c|c}\n'  #
                '  \\toprule\n'  #
                '  \\multicolumn{3}{c|}{\\textbf{Test Case}} & \\textbf{Run} & '
                '\\textbf{All Evaluations} \\\\\n'  #
            )
            for job_name in self.all_jobs_sorted:
                job_name_prefix = os.path.dirname(job_name)
                job_name_last_item = os.path.basename(job_name)

                job_results = self.evaluation_results[job_name]
                KEY_JOB_ESTIMATOR_AND_CONSOLE = 'job_estimator_and_console'
                estimator_and_console_success = (
                    KEY_JOB_ESTIMATOR_AND_CONSOLE in job_results
                    and job_results[KEY_JOB_ESTIMATOR_AND_CONSOLE] == 0)

                evaluation_success = True
                for evaluation_name, evaluation_result in job_results.iteritems(
                ):
                    if evaluation_name != KEY_JOB_ESTIMATOR_AND_CONSOLE:
                        evaluation_success = (evaluation_success
                                              and evaluation_result == 0)

                output_file.write('  \\midrule\n')
                output_file.write(
                    '  \\ref{%s} & %s & %s' %
                    (self.job_path_to_id[job_name],
                     self._experiment_name_to_title(job_name_prefix),
                     self._job_name_to_title(job_name_last_item)))

                color = 'NavyBlue' if (
                    estimator_and_console_success) else 'red'
                text = 'Completed' if (
                    estimator_and_console_success) else 'Failure'
                output_file.write(' & {\\color{%s}%s} & ' % (color, text))
                if estimator_and_console_success:
                    color = 'NavyBlue' if evaluation_success else 'red'
                    text = 'Success' if evaluation_success else 'Failure'
                    output_file.write('{\\color{%s}%s}' % (color, text))
                else:
                    output_file.write('(Not run)')
                output_file.write('\\\\\n')
            output_file.write(  #
                '  \\bottomrule\n'  #
                '\\end{longtabu}\n'  #
            )

    def _write_report_for_job(self, job, last_job_prefix):
        job_path = os.path.join(self.results_folder, job)
        job_prefix = os.path.dirname(job_path)
        job_report_path = os.path.realpath(
            os.path.join(job_path, 'job_report.tex'))
        with open(job_report_path, 'w+') as job_report:
            job_name = os.path.basename(job)
            if last_job_prefix == job_prefix:
                job_report.write('\\pagebreak\n')
            job_report.write('\\subsection{%s}\n' %
                             (self._job_name_to_title(job_name)))
            job_report.write(
                '\\label{%s}\n' % sanitize(self.job_path_to_id[job]))

            job_report.write('\\begin{tabular}{lcr}\n'  #
                             '  \\toprule\n'  #
                             '  \\textbf{Name} & \\textbf{Status} & '
                             '\\textbf{Exit code} \\\\\n'  #
                             '  \\midrule\n'  #
                             )

            job_results = self.evaluation_results[job]
            for evaluation_name, evaluation_result in job_results.iteritems():
                job_report.write(
                    '  %s & %s & %i\\\\\n' %
                    (sanitize(evaluation_name), '{\\color{NavyBlue}Success}'
                     if evaluation_result == 0 else '{\\color{red}Failure}',
                     evaluation_result))
            job_report.write('  \\bottomrule\n'  #
                             '\\end{tabular}\n'  #
                             )

            # Get all yamls in path.
            all_yamls_in_job = self._get_all_files_filtered_in_job(
                job_path, '*.yaml')
            all_pdfs_in_job = self._get_all_files_filtered_in_job(
                job_path, '*.pdf')
            all_files_in_job = all_yamls_in_job + all_pdfs_in_job

            if len(all_files_in_job) > 1:
                for idx, path in enumerate(all_files_in_job):
                    file_name = os.path.basename(path)
                    if file_name == 'job.yaml':
                        all_files_in_job[idx] = all_files_in_job[0]
                        all_files_in_job[0] = path
                    elif file_name == 'console_commands.yaml':
                        all_files_in_job[idx] = all_files_in_job[1]
                        all_files_in_job[1] = path

            # Filter out some files from list.
            all_files_in_job = [
                file_name for file_name in all_files_in_job
                if os.path.basename(file_name) not in
                ['sensors.yaml', 'job_summary.yaml']
            ]

            if len(all_files_in_job) > 2:
                all_files_in_job[2:] = sorted(all_files_in_job[2:])

            last_path = []
            for gen_file in all_files_in_job:
                file_name = os.path.basename(gen_file)
                path = os.path.dirname(gen_file).split('/')
                title_output, last_path = \
                        self._get_section_header_string_for_element(
                                job_name, path, file_name, last_path)
                job_report.write(title_output)
                if file_name == 'console_commands.yaml':
                    job_report.write(
                        yaml_outputter.write_console_commands_yaml(gen_file))
                elif file_name == 'job.yaml':
                    job_report.write(yaml_outputter.write_job_yaml(gen_file))
                elif file_name == 'errors.yaml':
                    job_report.write(
                        yaml_outputter.write_errors_yaml(gen_file))
                elif file_name.split('.')[-1] == 'yaml':
                    job_report.write(
                        yaml_outputter.write_generic_yaml_file(gen_file))
                elif file_name.split('.')[-1] == 'pdf':
                    real_pdf_path = os.path.realpath(gen_file)
                    job_report.write(
                        '\\includegraphics[width=0.9\\textwidth]{%s}\n' %
                        real_pdf_path)
        # Return the job prefix so that the outer loop can pass this in when
        # writing the next job. This is necessary to avoid a pagebreak just
        # after the section (= experiment) title.
        return job_prefix

    def _file_name_to_title(self, file_name):
        if file_name == 'job.yaml':
            return 'Estimator/Job Description (job.yaml)'
        elif file_name == 'console_commands.yaml':
            return 'Maplab Console Commands (console_commands.yaml)'
        return file_name
