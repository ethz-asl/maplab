import os
import yaml


class EvaluationRun(object):
    def __init__(self, path):
        self.path = path
        self.timestamp = os.path.basename(path)
        self.results_overview = yaml.load(
            open(os.path.join(path, 'evaluation_results.yaml')))
        self.jobs = {}
        for key, _ in self.results_overview.iteritems():
            self.jobs['_'.join(key.split('_')[2:])] = key
