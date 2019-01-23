from __future__ import print_function

import os

from collections import Counter
from pddlstream.utils import INF, read_pickle, ensure_dir, write_pickle, get_python_version


DATA_DIR = 'statistics/py{:d}/'

# TODO: ability to "burn in" streams by sampling artificially to get better estimates

def safe_ratio(numerator, denominator, undefined=None):
    if denominator == 0:
        return undefined
    return float(numerator) / denominator

def geometric_cost(cost, p):
    return safe_ratio(cost, p, undefined=INF)

##################################################

# TODO: write to a "local" folder containing temp, data2, data3, visualizations

def get_data_path(stream_name):
    data_dir = DATA_DIR.format(get_python_version())
    file_name = '{}.pkl'.format(stream_name)
    return os.path.join(data_dir, file_name)

def load_data(pddl_name):
    filename = get_data_path(pddl_name)
    if not os.path.exists(filename):
        return {}
    data = read_pickle(filename)
    #print('Loaded:', filename)
    return data

def load_stream_statistics(externals):
    if not externals:
        return
    pddl_name = externals[0].pddl_name # TODO: ensure the same
    # TODO: fresh restart flag
    data = load_data(pddl_name)
    for external in externals:
        if external.name in data:
            external.load_statistics(data[external.name])

##################################################

def dump_online_statistics(externals):
    print('\nLocal External Statistics')
    overall_calls = 0
    overall_overhead = 0
    for external in externals:
        external.dump_online()
        overall_calls += external.online_calls
        overall_overhead += external.online_overhead
    print('Overall calls: {} | Overall overhead: {:.3f}'.format(overall_calls, overall_overhead))

def dump_total_statistics(externals):
    print('\nTotal External Statistics')
    for external in externals:
        external.dump_total()
        # , external.get_effort()) #, data[external.name])

##################################################

def merge_data(external, previous_data):
    # TODO: compute distribution of successes given feasible
    # TODO: can estimate probability of success given feasible
    # TODO: single tail hypothesis testing (probability that came from this distribution)
    distribution = []
    for instance in external.instances.values():
        if instance.results_history:
            # attempts = len(instance.results_history)
            # successes = sum(map(bool, instance.results_history))
            # print(instance, successes, attempts)
            # TODO: also first attempt, first success
            last_success = -1
            for i, results in enumerate(instance.results_history):
                if results:
                    distribution.append(i - last_success)
                    # successful = (0 <= last_success)
                    last_success = i
    combined_distribution = previous_data.get('distribution', []) + distribution
    # print(external, distribution)
    # print(external, Counter(combined_distribution))
    # TODO: count num failures as well
    # Alternatively, keep metrics on the lower bound and use somehow
    # Could assume that it is some other distribution beyond that point
    return {
        'calls': external.total_calls,
        'overhead': external.total_overhead,
        'successes': external.total_successes,
        'distribution': combined_distribution,
    }
    # TODO: make an instance method

def write_stream_statistics(externals, verbose):
    # TODO: estimate conditional to affecting history on skeleton
    # TODO: estimate conditional to first & attempt and success
    # TODO: relate to success for the full future plan
    # TODO: Maximum Likelihood Exponential - average (biased in general)
    if not externals:
        return
    if verbose:
        #dump_online_statistics(externals)
        dump_total_statistics(externals)
    pddl_name = externals[0].pddl_name # TODO: ensure the same
    previous_data = load_data(pddl_name)
    data = {}
    for external in externals:
        if not hasattr(external, 'instances'):
            continue # TODO: SynthesizerStreams
        #total_calls = 0 # TODO: compute these values
        previous_statistics = previous_data.get(external.name, {})
        data[external.name] = merge_data(external, previous_statistics)

    filename = get_data_path(pddl_name)
    ensure_dir(filename)
    write_pickle(filename, data)
    if verbose:
        print('Wrote:', filename)

##################################################

def hash_object(evaluations, obj):
    # TODO: hash an object by the DAG of streams that produced it
    # Use this to more finely estimate the parameters of a stream
    # Can marginalize over conditional information to recover the same overall statistics
    # Can also apply this directly to domain facts
    raise NotImplementedError()

##################################################

class PerformanceInfo(object):
    def __init__(self, p_success, overhead):
        if p_success is not None:
            assert 0 <= p_success <= 1
        if overhead is not None:
            # TODO: overhead should never be zero (always some tiny overhead)
            assert 0 <= overhead
        self.p_success = p_success
        self.overhead = overhead

class Performance(object):
    def __init__(self, name, info):
        self.name = name.lower()
        self.info = info
        self.initial_calls = 0
        self.initial_overhead = 0.
        self.initial_successes = 0
        # TODO: online learning vs offline learning
        self.online_calls = 0
        self.online_overhead = 0.
        self.online_successes = 0

    @property
    def total_calls(self):
        return self.initial_calls + self.online_calls

    @property
    def total_overhead(self):
        return self.initial_overhead + self.online_overhead

    @property
    def total_successes(self):
        return self.initial_successes + self.online_successes

    def load_statistics(self, statistics):
        self.initial_calls = statistics['calls']
        self.initial_overhead = statistics['overhead']
        self.initial_successes = statistics['successes']

    def update_statistics(self, overhead, success):
        self.online_calls += 1
        self.online_overhead += overhead
        self.online_successes += success

    def _estimate_p_success(self, reg_p_success=1., reg_calls=1):
        # TODO: use prior from info instead?
        return safe_ratio(self.total_successes + reg_p_success * reg_calls,
                          self.total_calls + reg_calls,
                          undefined=reg_p_success)

    def _estimate_overhead(self, reg_overhead=1e-6, reg_calls=1):
        # TODO: use prior from info instead?
        return safe_ratio(self.total_overhead + reg_overhead * reg_calls,
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

    def dump_total(self):
        print('External: {} | n: {:d} | p_success: {:.3f} | overhead: {:.3f}'.format(
            self.name, self.total_calls, self.get_p_success(), self.get_overhead()))

    def dump_online(self):
        if not self.online_calls:
            return
        print('External: {} | n: {:d} | p_success: {:.3f} | mean overhead: {:.3f} | overhead: {:.3f}'.format(
            self.name, self.online_calls,
            safe_ratio(self.online_successes, self.online_calls),
            safe_ratio(self.online_overhead, self.online_calls),
            self.online_overhead))

##################################################

# TODO: cannot easily do Bayesian hypothesis testing because might never receive ground truth when empty
# In some cases, the stream does finish though

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

# Previously in Instance
# def get_belief(self):
#     #return 1.
#     #prior = self.external.prior
#     prior = 1. - 1e-2
#     n = self.num_calls
#     p_obs_given_state = self.external.get_p_success()
#     p_state = prior
#     for i in range(n):
#         p_nobs_and_state = (1-p_obs_given_state)*p_state
#         p_nobs_and_nstate = (1-p_state)
#         p_nobs = p_nobs_and_state + p_nobs_and_nstate
#         p_state = p_nobs_and_state/p_nobs
#     return p_state

# def update_belief(self, success):
#     # Belief that remaining sequence is non-empty
#     # Belief only degrades in this case
#     nonempty = 0.9
#     p_success_nonempty = 0.5
#     if success:
#         p_success = p_success_nonempty*nonempty
#     else:
#         p_success = (1-p_success_nonempty)*nonempty + (1-nonempty)

#def get_p_success(self):
    #p_success_belief = self.external.get_p_success()
    #belief = self.get_belief()
    #return p_success_belief*belief
    # TODO: use the external as a prior
    # TODO: Bayesian estimation of likelihood that has result
    # Model hidden state of whether has values or if will produce values?
    # TODO: direct estimation of different buckets in which it will finish
    # TODO: we have samples from the CDF or something

#def get_p_success(self):
#    return self.external.get_p_success()
#
#def get_overhead(self):
#    return self.external.get_overhead()
