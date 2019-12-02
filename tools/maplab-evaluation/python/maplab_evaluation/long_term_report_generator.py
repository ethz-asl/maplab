#!/usr/bin/env python

import argparse
import os

import yaml

from evaluation_tools.catkin_utils import catkinFindSrc

from maplab_evaluation.evaluation_run import EvaluationRun
from maplab_evaluation.plot_generator import overview_plot, error_history_plot
from maplab_evaluation.report_common \
        import format_date, generate_random_string, get_commit_info, sanitize
from maplab_evaluation.report_generator_base import ReportGeneratorBase


class LongTermReportGenerator(ReportGeneratorBase):
    def __init__(self, results_folders, max_number_of_results_to_use,
                 output_folder):
        ReportGeneratorBase.__init__(self)
        self.report_file = 'long_term_report.tex'
        self.output_folder = output_folder

        # Get all results folders.
        self.all_evaluation_runs = []
        self.all_individual_experiments = set()
        for results_folder in results_folders:
            if os.path.isdir(results_folder):
                self._check_folder_for_results(results_folder)

        # Sort all runs by timestamp and remove oldest ones.
        self.all_evaluation_runs = sorted(
            self.all_evaluation_runs, key=lambda obj: obj.timestamp)
        if len(self.all_evaluation_runs) > max_number_of_results_to_use:
            self.all_evaluation_runs = self.all_evaluation_runs[
                -max_number_of_results_to_use:]

        # Obtain all jobs in all runs.
        for evaluation_run in self.all_evaluation_runs:
            for job_name, _ in evaluation_run.jobs.iteritems():
                self.all_individual_experiments.add(job_name)
        self.all_individual_experiments = sorted(
            self.all_individual_experiments)

        self._create_random_ids_for_all_jobs(self.all_individual_experiments)

    def _check_folder_for_results(self, folder):
        if not os.path.isdir(folder):
            return
        evaluation_candidates = os.listdir(folder)
        for candiate_name in evaluation_candidates:
            candidate_folder = os.path.join(folder, candiate_name)
            if os.path.isfile(
                    os.path.join(candidate_folder, 'evaluation_results.yaml')):
                evaluation_run = EvaluationRun(candidate_folder)
                self.all_evaluation_runs.append(evaluation_run)
            else:
                self._check_folder_for_results(candidate_folder)

    def generate_report(self):
        self._generate_report(
            output_folder=self.output_folder, latex_main_file=self.report_file)

    def _write_base_structure(self):
        template_file_path = os.path.join(
            catkinFindSrc('maplab_evaluation'), 'latex', 'template.tex')
        with open(template_file_path, 'r') as template_file:
            template = template_file.read()

        content_summary = '\\input{%s}\n' % os.path.realpath(
            'long_term_summary.tex')

        last_job_prefix = None
        content_all_jobs = ''
        for job_name in self.all_individual_experiments:
            job_prefix = os.path.dirname(job_name)
            if job_prefix != last_job_prefix:
                last_job_prefix = job_prefix
                content_all_jobs += '\\pagebreak\n'
                content_all_jobs += '\n\\section{%s}\n' % (
                    sanitize(job_prefix))
            content_all_jobs += self._write_output_for_job(job_name)

        with open(self.report_file, 'w+') as output_file:
            output_file.write(
                template.replace('SUMMARYCONTENT', content_summary).replace(
                    'PERJOBCONTENT', content_all_jobs))

    def _write_table_header(self, evaluation_runs):
        output = ''
        for run in evaluation_runs:
            output += '& \\rotatebox{90}{\\textbf{%s}}' % sanitize(
                format_date(run.timestamp))
        output += '\\\\\n'
        return output

    def _write_summary(self):
        with open('long_term_summary.tex', 'w+') as output_file:
            summary_plot_path = overview_plot([
                format_date(obj.timestamp) for obj in self.all_evaluation_runs
            ], [obj.results_overview for obj in self.all_evaluation_runs])
            output_file.write('\\includegraphics[width=\\textwidth]{%s}\n' %
                              summary_plot_path)

            # Write overview table.
            output_file.write('\\begin{longtabu} to \\textwidth {l|')
            output_file.write('X[c]' * len(self.all_evaluation_runs))
            output_file.write('}\n'  #
                              '\\toprule\n')
            output_file.write(
                self._write_table_header(self.all_evaluation_runs))
            output_file.write('\\midrule\n')

            # Loop through all tests.
            for job in self.all_individual_experiments:
                output_file.write('  \\ref{%s}\n' % self.job_path_to_id[job])
                for run in self.all_evaluation_runs:
                    if job in run.jobs:
                        job_run_results = run.results_overview[run.jobs[job]]
                        KEY_JOB_ESTIMATOR_AND_CONSOLE = \
                                'job_estimator_and_console'
                        estimator_and_console_success = (
                            KEY_JOB_ESTIMATOR_AND_CONSOLE in job_run_results
                            and
                            job_run_results[KEY_JOB_ESTIMATOR_AND_CONSOLE] == 0
                        )
                        evaluation_success = True
                        for evaluation_name, evaluation_result in \
                                job_run_results.iteritems():
                            if evaluation_name != KEY_JOB_ESTIMATOR_AND_CONSOLE:
                                evaluation_success = (evaluation_success and
                                                      evaluation_result == 0)

                        if not estimator_and_console_success:
                            output_file.write(
                                '  & {\\leavevmode\\color{Red}\\xmark}\n')
                        elif not evaluation_success:
                            output_file.write(
                                '  & {\\leavevmode\\color{Orange}'
                                '\\semicircle}\n')
                        else:
                            output_file.write(
                                '  & {\\leavevmode\\color{ForestGreen}'
                                '\\cmark}\n')
                    else:
                        output_file.write('  & --\n')
                output_file.write('  \\\\\n')

            output_file.write('\\bottomrule\n'  #
                              '\\end{longtabu}\n')
            output_file.write(
                '--: not run or no results, {\\color{Red}\\xmark}: run failed, '
                '{\\color{Orange}\\semicircle}: evaluation failed, '
                '{\\color{ForestGreen}\\cmark}: everthing succeeded\n\n')

    def _write_output_for_job(self, job_name):
        job_base_name = os.path.basename(job_name)
        output = '\\subsection{%s}\n' % sanitize(job_base_name)
        output += '\\label{%s}\n' % self.job_path_to_id[job_name]
        all_yaml_files_set = set()
        for evaluation_run in self.all_evaluation_runs:
            if job_name in evaluation_run.jobs:
                job_path = os.path.join(evaluation_run.path,
                                        evaluation_run.jobs[job_name])
                all_yamls_in_job = self._get_all_files_filtered_in_job(
                    job_path, '*.yaml')
                for yaml_file in all_yamls_in_job:
                    all_yaml_files_set.add(
                        os.path.relpath(yaml_file, job_path))

        all_yaml_files = [element for element in all_yaml_files_set]
        if all_yamls_in_job:
            for idx, yaml_file in enumerate(all_yaml_files):
                file_name = os.path.basename(yaml_file)
                if file_name == 'job_summary.yaml':
                    all_yaml_files[idx] = all_yaml_files[0]
                    all_yaml_files[0] = 'job_summary.yaml'

        if len(all_yaml_files) > 1:
            all_yaml_files[1:] = sorted(all_yaml_files[1:])

        file_name_to_label_map = {}
        if len(self.all_evaluation_runs) >= 2:
            if len(self.all_evaluation_runs) > 5:
                evaluation_runs_in_overview = self.all_evaluation_runs[-5:]
            else:
                evaluation_runs_in_overview = self.all_evaluation_runs

            output += '\\subsubsection{Overview}\n'
            output += '\\begin{longtabu} to \\textwidth{l'
            output += 'X[r]' * (len(evaluation_runs_in_overview) - 1)
            output += '}\n'
            output += '\\toprule\n'
            for idx, evaluation_run in enumerate(evaluation_runs_in_overview):
                if idx == 0:
                    continue
                output += (
                    '& \\rotatebox{90}{\\parbox{4cm}{'
                    '{\\color{white}to }\\textbf{%s}\\\\to \\textbf{%s}}}') % (
                        sanitize(
                            format_date(
                                evaluation_runs_in_overview[idx - 1].timestamp,
                                new_line_between_date_and_time=False)),
                        sanitize(
                            format_date(
                                evaluation_run.timestamp,
                                new_line_between_date_and_time=False)))
            output += '\\\\\n'
            for yaml_file in all_yaml_files:
                if yaml_file not in file_name_to_label_map:
                    file_name_to_label_map.update({
                        yaml_file:
                        generate_random_string(length=10)
                    })
                if os.path.basename(yaml_file) == 'errors.yaml':
                    output += self._job_overview_table_print_errors_yaml(
                        evaluation_runs_in_overview, job_name, yaml_file,
                        file_name_to_label_map)
            output += '\\bottomrule\n'
            output += '\\end{longtabu}\n'

        last_path = []
        for yaml_file in all_yaml_files:
            generated_output = None
            title = None
            if os.path.basename(yaml_file) == 'job_summary.yaml':
                title = 'Revision info'
                generated_output = '\\begin{longtabu} to \\textwidth {lX}'
                generated_output += '\\toprule\n'
                generated_output += '\\textbf{Run} & \\textbf{Revision} \\\\\n'

                generated_output += '\\midrule\n'
                for run in self.all_evaluation_runs:
                    generated_output += '  %s & ' % sanitize(
                        format_date(run.timestamp))
                    if job_name not in run.jobs:
                        generated_output += '(Not run)\\\\\n'
                        continue
                    relative_job_path = run.jobs[job_name]
                    job_yaml_path = os.path.join(run.path, relative_job_path,
                                                 'job.yaml')
                    summary_path = os.path.join(run.path, relative_job_path,
                                                'job_summary.yaml')
                    if os.path.isfile(job_yaml_path) and os.path.isfile(
                            summary_path):
                        job_dict = yaml.load(open(job_yaml_path))
                        job_summary = yaml.load(open(summary_path))

                        revision = str(job_summary['executable']['rev'])
                        generated_output += 'Revision: %s\n\n' % sanitize(
                            revision)

                        if 'app_package_name' in job_dict:
                            generated_output += sanitize(
                                get_commit_info(job_dict['app_package_name'],
                                                revision))
                    else:
                        generated_output += '(No revision info available)'

                    generated_output += '\\\\\n'

                generated_output += '\\bottomrule\n'
                generated_output += '\\end{longtabu}\n'
            elif os.path.basename(yaml_file) == 'errors.yaml':
                generated_plots = error_history_plot(self.all_evaluation_runs,
                                                     job_name, yaml_file)
                generated_output = ''
                for plot in generated_plots:
                    generated_output += (
                        '\\includegraphics[width=0.9\\textwidth]{%s}\n\n' %
                        os.path.realpath(plot))

            if generated_output is None:
                continue

            path = os.path.dirname(os.path.join(job_path,
                                                yaml_file)).split('/')
            title_output, last_path = (
                self._get_section_header_string_for_element(
                    job_base_name, path,
                    os.path.basename(yaml_file)
                    if title is None else title, last_path))
            output += title_output
            if yaml_file in file_name_to_label_map:
                output += '\\label{%s}\n' % file_name_to_label_map[yaml_file]
            output += generated_output

        return output

    def _job_overview_table_print_errors_yaml(self, evaluation_results,
                                              job_name, yaml_file,
                                              file_name_to_label_map):
        output = '\\midrule\n'
        output += ('  \\multicolumn{%i}{l}{\\textbf{'
                   '\\hyperref[%s]{%s}}}\\\\\n' %
                   (len(evaluation_results), file_name_to_label_map[yaml_file],
                    sanitize(yaml_file)))
        at_least_one_value_printed_for_job = False
        for label, outer_tag, inner_tag in zip([
                'Position mean [m]',
                'Position RMSE [m]',
                'Max position error [m]',
                'Orientation mean [rad]',
                'Orientation RMSE [rad]',
                'Max orientation error [rad]',
        ], [
                value for value in ['position_errors', 'orientation_errors']
                for _ in range(3)
        ], ['mean', 'rmse', 'max'] * 2):
            at_least_one_value_printed_in_row = False
            output_of_row = '  %s' % sanitize(label)
            for idx, run_2 in enumerate(evaluation_results):
                value_printed = False
                if idx == 0:
                    # Ignore first value as there is no previous value available
                    # in the list to compare to.
                    continue
                run_1 = evaluation_results[idx - 1]
                file_path_1 = ''
                file_path_2 = ''
                if job_name in run_1.jobs:
                    file_path_1 = os.path.join(run_1.path,
                                               run_1.jobs[job_name], yaml_file)
                if job_name in run_2.jobs:
                    file_path_2 = os.path.join(run_2.path,
                                               run_2.jobs[job_name], yaml_file)
                if os.path.isfile(file_path_1) and os.path.isfile(file_path_2):
                    errors_1 = yaml.load(open(file_path_1))
                    errors_2 = yaml.load(open(file_path_2))
                    if (outer_tag in errors_1
                            and inner_tag in errors_1[outer_tag]
                            and outer_tag in errors_2
                            and inner_tag in errors_2[outer_tag]):
                        if errors_1[outer_tag][inner_tag] != 0:
                            ratio_change = ((errors_2[outer_tag][inner_tag] -
                                             errors_1[outer_tag][inner_tag]) /
                                            errors_1[outer_tag][inner_tag])
                            color = 'black'
                            if ratio_change > 0:
                                color = 'red'
                                if (idx == len(evaluation_results) - 1
                                        and ratio_change > 0.05):
                                    # Last entry: print warning for jenkins
                                    # parser that result got worse.
                                    print(
                                        'Evaluation job progression: '
                                        'performance of job "%s" in "%s" for '
                                        'label "%s" decreased by %.2f%% '
                                        'between the last two runs.' %
                                        (job_name, yaml_file, label,
                                         ratio_change * 100))
                            elif ratio_change < 0:
                                color = 'ForestGreen'
                            output_of_row += ('  & {\\leavevmode'
                                              '\\color{%s}%+.2f\\%%}') % (
                                                  color, ratio_change * 100)
                            value_printed = True
                            at_least_one_value_printed_in_row = True
                            at_least_one_value_printed_for_job = True
                if not value_printed:
                    output_of_row += '  & --'

            if at_least_one_value_printed_in_row:
                output += output_of_row
                output += '\\\\\n'

        if not at_least_one_value_printed_for_job:
            output += (
                '  \\multicolumn{%i}{l}{'
                '(Not enough runs to show progression. '
                'Need at least two consecutive runs to show any data.)}\\\\\n'
            ) % (len(evaluation_results))
        return output


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--results_folders', nargs='+', default=['./results'])
    parser.add_argument('--max_number_of_results_to_use', default=10, type=int)
    parser.add_argument('--output_folder', default='.')

    args = parser.parse_args()
    generator = LongTermReportGenerator(args.results_folders,
                                        args.max_number_of_results_to_use,
                                        args.output_folder)
    generator.generate_report()
