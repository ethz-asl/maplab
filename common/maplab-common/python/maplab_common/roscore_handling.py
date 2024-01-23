import signal
import subprocess
import sys
import rosgraph
import psutil


def kill_child_processes(parent_pid, sig=signal.SIGTERM):
    try:
        parent = psutil.Process(parent_pid)
        print(parent)
    except psutil.NoSuchProcess:
        print("parent process not existing")
        return
    children = parent.children(recursive=True)
    print(children)
    for process in children:
        print("try to kill child process: " + str(process))
        process.send_signal(sig)


class Roscore(object):
    """
    roscore wrapped into a subprocess.
    Singleton implementation prevents from creating more than one instance.
    https://answers.ros.org/question/215600/how-can-i-run-roscore-from-python/
    """
    __initialized = False

    def __init__(self):
        if Roscore.__initialized:
            raise Exception(
                "You can't create more than 1 instance of Roscore.")
        self.roscore_process = None
        self.roscore_pid = -1
        self.already_running = False
        Roscore.__initialized = True

    def run(self):
        try:
            self.already_running = rosgraph.is_master_online()
            if self.already_running:
                print('There already is a roscore up and running.')
                return
            self.roscore_process = subprocess.Popen(['roscore'])
            # pid of the roscore process (which has child processes)
            self.roscore_pid = self.roscore_process.pid
        except OSError as e:
            sys.stderr.write('roscore could not be run')
            raise e

    def terminate(self):
        if self.already_running:
            print('There already is a roscore up and running.')
            print('The python roscore handler will not kill it.')
            print('Please do so manually if desired.')
            return
        print(
            "try to kill child pids of roscore pid: " + str(self.roscore_pid))
        kill_child_processes(self.roscore_pid)
        self.roscore_process.terminate()
        self.roscore_process.wait()  # important to prevent from zombie process
        Roscore.__initialized = False
