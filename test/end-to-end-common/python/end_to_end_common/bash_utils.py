#!/usr/bin/env python

import os
import shlex
import subprocess as sp
  
def run(cmd, dry_run=True):
  if dry_run:
    print("Would normally run:", cmd)
    return
  print("Running:", cmd)
  
  args = shlex.split(cmd)
  proc = sp.Popen(cmd, stdin=sp.PIPE, stdout=sp.PIPE, stderr=sp.STDOUT, 
                  close_fds=True)
  exit_code = proc.wait()
  output = proc.stdout.read()
  if exit_code != 0:
    raise Exception("Cmd '%s' returned nozero exit code : %d with output:\n %s" %
                    (cmd, exit_code, output))


def run(cmd, dry_run=False):
  if dry_run:
    print("Would normally run:", cmd)
    return
  print("Running:", cmd)
  import subprocess
  import shlex
  args = shlex.split(cmd)
  proc = subprocess.Popen(args)
  exit_code = proc.wait()
  if exit_code != 0:
    raise Exception("Cmd '%s' returned nozero exit code : %d" %
                    (cmd, exit_code))
def create_path(path):
  if not os.path.exists(os.path.dirname(path)):
    try:
      os.makedirs(os.path.dirname(path))
    except OSError as exc:  # Guard against race condition
      if exc.errno != errno.EEXIST:
        raise
