from __future__ import print_function

import os
from collections import defaultdict

from pddlstream.utils import INF, read_pickle, ensure_dir, write_pickle

import sys

class ActionInfo(object):
    def __init__(self, terminal=False, p_success=None, overhead=None):
        """
        :param terminal: Indicates the action may require replanning after use
        """
        self.terminal = terminal # TODO: infer from p_success?
        if self.terminal:
            self.p_success, self.overhead = 1e-3, 0
        else:
            self.p_success, self.overhead = 1, INF
        if p_success is not None:
            self.p_success = p_success
        if overhead is not None:
            self.overhead = overhead
        # TODO: should overhead just be cost here then?


def get_action_info(action_info):
    action_execution = defaultdict(ActionInfo)
    for name, info in action_info.items():
        action_execution[name] = info
    return action_execution

##################################################


DATA_DIR = 'data{:d}/'

def get_python_version():
    return sys.version_info[0]

def get_data_directory():
    return DATA_DIR.format(get_python_version())


def get_stream_data_filename(stream_name):
    return os.path.join(get_data_directory(), '{}.pp'.format(stream_name))


def load_stream_statistics(stream_name, externals):
    # TODO: fresh restart flag
    filename = get_stream_data_filename(stream_name)
    if not os.path.exists(filename):
        return
    data = read_pickle(filename)
    for external in externals:
        if external.name in data:
            statistics = data[external.name]
            external.total_calls += statistics['calls']
            external.total_overhead += statistics['overhead']
            external.total_successes += statistics['successes']


def write_stream_statistics(stream_name, externals, verbose):
    if not externals:
        return
    if verbose:
        print('\nLocal External Statistics')
        overall_calls = 0
        overall_overhead = 0
        for external in externals:
            external.dump_local()
            overall_calls += external.online_calls
            overall_overhead += external.online_overhead
        print('Overall calls: {} | Overall overhead: {}'.format(overall_calls, overall_overhead))

        print('\nTotal External Statistics')
        for external in externals:
            external.dump_total()
        # , external.get_effort()) #, data[external.name])

    data = {}
    for external in externals:
        data[external.name] = {
            'calls': external.total_calls,
            'overhead': external.total_overhead,
            'successes': external.total_successes,
        }
    filename = get_stream_data_filename(stream_name)
    ensure_dir(filename)
    write_pickle(filename, data)
    if verbose:
        print('Wrote:', filename)

##################################################

def compute_ratio(numerator, denomenator, undefined=None):
    if denomenator == 0:
        return undefined
    return float(numerator) / denomenator

def geometric_cost(cost, p):
    return compute_ratio(cost, p, undefined=INF)

class Performance(object):
    # TODO: estimate conditional to affecting history on skeleton
    # TODO: estimate conditional to first & attempt and success

    # Estimate probability that will generate result
    # Need to produce belief that has additional samples

    # P(Success | Samples) = estimated parameter
    # P(Success | ~Samples) = 0
    # T(Samples | ~Samples) = 0
    # T(~Samples | Samples) = 1-p

    # TODO: estimate a parameter conditioned on successful streams?
    # Need a transition fn as well because generating a sample might change state
    # Problem with estimating prior. Don't always have data on failed streams


    # Goal: estimate P(Success | History)
    # P(Success | History) = P(Success | Samples) * P(Samples | History)

    def __init__(self, name, info):
        self.name = name.lower()
        self.info = info
        self.total_calls = 0
        self.total_overhead = 0
        self.total_successes = 0
        self.online_calls = 0
        self.online_overhead = 0
        self.online_success = 0

    def update_statistics(self, overhead, success):
        self.total_calls += 1
        self.total_overhead += overhead
        self.total_successes += success
        self.online_calls += 1
        self.online_overhead += overhead
        self.online_success += success

    #def is_first_call(self): # TODO: use in streams
    #    return self.online_calls == 0
    #
    #def has_previous_success(self):
    #    return self.online_success != 0

    def _estimate_p_success(self, reg_p_success=1, reg_calls=1):
        # TODO: use prior from info instead?
        return compute_ratio(self.total_successes + reg_p_success*reg_calls,
                             self.total_calls + reg_calls,
                             undefined=reg_p_success)

    def _estimate_overhead(self, reg_overhead=0, reg_calls=1):
        # TODO: use prior from info instead?
        return compute_ratio(self.total_overhead + reg_overhead*reg_calls,
                             self.total_calls + reg_calls,
                             undefined=reg_overhead)

    def get_p_success(self):
        if self.info.p_success is None:
            return self._estimate_p_success()
        return self.info.p_success

    def get_overhead(self):
        if self.info.overhead is None:
            return self._estimate_overhead()
        return self.info.overhead

    def get_effort(self):
        return geometric_cost(self.get_overhead(), self.get_p_success())

    def dump_total(self):
        print('External: {} | n: {:d} | p_success: {:.3f} | overhead: {:.3f}'.format(
            self.name, self.total_calls,
            self.get_p_success(), self.get_overhead()))

    def dump_local(self):
        if not self.online_calls:
            return
        print('External: {} | n: {:d} | p_success: {:.3f} | mean overhead: {:.3f} | overhead: {:.3f}'.format(
            self.name, self.online_calls,
            compute_ratio(self.online_success, self.online_calls),
            compute_ratio(self.online_overhead, self.online_calls),
            self.online_overhead))
