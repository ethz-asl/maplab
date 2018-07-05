#!/usr/bin/env python

import os

import yaml

from maplab_evaluation.report_common import get_commit_info, sanitize


def format_command(command, highlight_first_n_words=0):
    command_words = command.split()
    for idx, word in enumerate(command_words):
        word = sanitize(word)
        if idx < highlight_first_n_words:
            word = '{\\color{MidnightBlue}%s}' % word
        else:
            if word[0] == '-':
                # Color flags up to '=' sign.
                word_segments = word.split('=')
                word = '{\\color{PineGreen}%s}' % word_segments[0]
                if len(word_segments) > 1:
                    word += '=' + '='.join(word_segments[1:])

        command_words[idx] = word

    return ' '.join(command_words)


def write_console_commands_yaml(console_commands_path):
    console_commands = yaml.load(open(console_commands_path))
    output = "\\begin{ttblock}\\begin{enumerate}\n"
    for command in console_commands['commands']:
        output += "  \\item %s\n" % format_command(
            command, highlight_first_n_words=1)
    output += "\\end{enumerate}\\end{ttblock}\n"
    return output


def write_generic_yaml_file(yaml_file):
    output = "\\begin{lstlisting}[breaklines]\n"
    with open(yaml_file, 'r') as yaml_file_opened:
        output += yaml_file_opened.read()
    output += "\\end{lstlisting}\n"
    return output


def write_errors_yaml(yaml_file):
    error_dict = yaml.load(open(yaml_file))
    output = '\\begin{tabular}{l|ccc|ccc}\n'
    output += '  \\toprule\n'
    output += '  & \\multicolumn{3}{c|}{\\textbf{Position [m]}} & ' \
        '\\multicolumn{3}{c}{\\textbf{Orientation [rad]}}\\\\\n'
    output += '  & measured & threshold & & measured & threshold & \\\\\n'
    output += '  \\midrule\n'

    OUTPUT_THRESHOLD_OK = ' & {\\color{NavyBlue}success}'
    OUTPUT_THRESHOLD_NOT_OK = ' & {\\color{red}failure}'
    OUTPUT_NO_THRESHOLD = '-- &'

    for print_name in ['Mean', 'RMSE', 'Min', 'Max']:
        name_in_yaml = print_name.lower()
        output += '  \\textbf{%s}' % print_name
        for type_in_yaml in ['position_errors', 'orientation_errors']:
            output += ' & %f & ' % error_dict[type_in_yaml][name_in_yaml]
            if ('thresholds' in error_dict[type_in_yaml] and
                    name_in_yaml in error_dict[type_in_yaml]['thresholds']):
                output += str(
                    error_dict[type_in_yaml]['thresholds'][name_in_yaml])
                if (error_dict[type_in_yaml][name_in_yaml] <
                        error_dict[type_in_yaml]['thresholds'][name_in_yaml]):
                    output += OUTPUT_THRESHOLD_OK
                else:
                    output += OUTPUT_THRESHOLD_NOT_OK
            else:
                output += OUTPUT_NO_THRESHOLD
        output += '\\\\\n'

    output += '  \\bottomrule\n'
    output += '\\end{tabular}\n'
    return output


def write_job_yaml(yaml_file):
    job_dict = yaml.load(open(yaml_file))

    output = '\\begin{description}'
    output += '  \\item[Job folder:] %s\n' % sanitize(
        os.path.realpath(os.path.dirname(yaml_file)))
    for name, key in zip(
        [  # pylint: disable=bad-continuation
            'Package name', 'Executable', 'Experiment name',
            'Experiment description', 'Experiment root folder',
            'Parameter file'
        ],
        [  # pylint: disable=bad-continuation
            'app_package_name',
            'app_executable',
            'experiment_filename',  #
            'description',
            'experiment_root_folder',
            'parameter_file'
        ]):
        if key in job_dict:
            output += '\\item[%s:] %s\n' % (sanitize(name),
                                            sanitize(str(job_dict[key])))

    job_summary_file = os.path.join(
        os.path.dirname(yaml_file), 'job_summary.yaml')
    output += '  \\item[Revision:]'
    if os.path.isfile(job_summary_file):
        job_summary = yaml.load(open(job_summary_file))
        revision = str(job_summary['executable']['rev'])
        output += ' %s\n' % sanitize(revision)

        if 'app_package_name' in job_dict:
            output += '  \\\\%s\n' % sanitize(
                get_commit_info(job_dict['app_package_name'], revision))
    else:
        output += 'Unknown (no job\\_summary.yaml available)\n'

    output += '\\item[Datasets:] {\\color{white}.}\\\\\n'
    output += '  \\begin{itemize}\n'
    for dataset_dict in job_dict['datasets']:
        output += '\\item Dataset \\inltt{%s}\\\\\n' % (
            sanitize(dataset_dict['name']))
        command = 'rosrun %s %s<NEWLINE>' % (
            sanitize(job_dict['app_package_name']),
            sanitize(job_dict['app_executable']))
        for name, value in dataset_dict['parameters'].iteritems():
            command += ' --%s=%s<NEWLINE>' % (name, value)
        command = format_command(command)
        command = command.replace('<NEWLINE>', '\\\\ \\hspace*{0.5cm}')

        output += '    Command:\n\\begin{ttblock}%s\\end{ttblock}\n\n' % command
        output += '    Additional dataset parameters (used for placeholder ' \
            'replacement and evaluation scripts):\n\n'
        output += '    \\begin{longtabu} to \\textwidth {X[1,l]X[2,l]}\n'
        output += '      \\toprule\n'
        output += '      \\textbf{Name} & \\textbf{Value} \\\\\n'
        for name, value in dataset_dict['additional_parameters'].iteritems():
            output += '      \\midrule\n'
            output += '      \\inltt{%s} & \\inltt{%s} \\\\\n' % (
                sanitize(name), sanitize(str(value)))
        output += '      \\bottomrule\n'
        output += '    \\end{longtabu}\n'
    output += '  \\end{itemize}\n'
    if 'evaluation_scripts' in job_dict:
        output += '  \\item[Evaluation scripts:] These are run after the ' \
            'estimator is done and the console has run.\n'
        output += '    \\begin{itemize}\n'
        for evaluation_dict in job_dict['evaluation_scripts']:
            output += '    \\item\\begin{description}\n'
            output += '      \\item[Name:] \\inltt{%s}\n' % sanitize(
                evaluation_dict['name'])
            if 'arguments' in evaluation_dict:
                output += '      \\item[Arguments:] \\inltt{%s}\n' % sanitize(
                    str(evaluation_dict['arguments']))
            output += '    \\end{description}\n'
        output += '    \\end{itemize}\n'

    output += '\\end{description}'
    return output
