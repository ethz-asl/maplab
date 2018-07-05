#!/usr/bin/env python

import random
import string  # pylint: disable=deprecated-module
import subprocess

from evaluation_tools.catkin_utils import catkinFindSrc


def sanitize(text):
    """Escapes characters that have a special meaning in latex."""
    text = text.replace('\\', '\\textbackslash{}')

    for c in '$%^{}#_&':
        text = text.replace(c, '\\' + c)

    # Allow some more line breaks to happen.
    text = text.replace('/', '\\slash{}')
    text = text.replace('=', '=\\allowbreak{}')
    text = text.replace('-', '-\\allowbreak{}')

    return text


def format_date(date, new_line_between_date_and_time=True):
    assert len(date) >= 15
    return (
        sanitize(date[6:8]) + '.' + sanitize(date[4:6]) + '.' + sanitize(
            date[0:4]) +
        ('\n'
         if new_line_between_date_and_time else ' ') + sanitize(date[9:11]) +
        ':' + sanitize(date[11:13]) + ':' + sanitize(date[13:15]))


def generate_random_string(length):
    # From
    # https://stackoverflow.com/questions/37675280/how-to-generate-a-random-string # pylint: disable=line-too-long
    return ''.join([random.choice(string.lowercase) for _ in xrange(length)])


def get_commit_info(
        package_name,
        revision,
        git_format='format:Date: %ad, author: %an\n\nCommit message: %s'):
    git_call = ('git show {} --quiet --pretty="{}"'.format(
        revision, git_format))
    package_src_folder = catkinFindSrc(package_name)
    process = subprocess.Popen(
        git_call,
        shell=True,
        cwd=package_src_folder,
        stdin=subprocess.PIPE,
        stdout=subprocess.PIPE)
    stdout, _ = process.communicate()
    return stdout
