#!/usr/bin/env python

from __future__ import print_function

import numpy as np

from collections import namedtuple
from copy import deepcopy

from examples.continuous_tamp.primitives import get_pose_gen, inverse_kin_fn, get_region_test, plan_motion, \
    tight, blocked, draw_state, get_random_seed, TAMPState, GROUND_NAME, GRASP, SUCTION_HEIGHT
from examples.continuous_tamp.viewer import ContinuousTMPViewer
from examples.discrete_tamp.viewer import COLORS
from pddlstream.algorithms.focused import solve_focused
from pddlstream.algorithms.incremental import solve_incremental
from pddlstream.language.generator import from_gen_fn, from_fn, from_test
from pddlstream.utils import user_input, read, INF, get_file_path, Profiler
from pddlstream.language.constants import print_solution

#Mode = namedtuple('Mode', ['grasp'])

def sample_forward(problem):
    def gen(mode1):
        if mode1 is not None:
            yield None
        else:
            # TODO: other object poses
            for block in problem.initial.block_poses:
                mode2 = {
                    'block': block,
                    'grasp': GRASP,
                }
                yield (mode2,)
    return gen

def sample_intersection(problem):
    def gen(mode1, mode2):
        if mode1 is None:
            grasp_mode = mode2
        elif mode2 is None:
            grasp_mode = mode1
        else:
            raise RuntimeError()
        print(grasp_mode)

        yield None
    return gen

def sample_connection(problem):
    def gen(mode, conf1, conf2):
        yield None
    return gen

##################################################

def pddlstream_from_tamp(tamp_problem):
    initial = tamp_problem.initial
    assert(not initial.holding) # is None

    domain_pddl = read(get_file_path(__file__, 'domain.pddl'))
    stream_pddl = read(get_file_path(__file__, 'stream.pddl'))
    constant_map = {}

    mode = None
    conf = {}
    conf.update(initial.robot_confs)
    conf.update(initial.block_poses)

    init = [
        ('CanMove'),
        ('Mode', mode),
        ('AtMode', mode),
        ('Conf', mode, conf),
        ('AtConf', conf),
    ]

    goal = ('AtGoal',) # Existential

    stream_map = {
        's-forward': from_gen_fn(sample_forward(tamp_problem)),
        's-intersection': from_gen_fn(sample_intersection(tamp_problem)),
        's-connection': from_gen_fn(sample_connection(tamp_problem)),
    }

    return domain_pddl, constant_map, stream_pddl, stream_map, init, goal

##################################################

def main(deterministic=False, unit_costs=True):
    np.set_printoptions(precision=2)
    if deterministic:
        seed = 0
        np.random.seed(seed)
    print('Seed:', get_random_seed())

    problem_fn = tight  # get_tight_problem | get_blocked_problem
    tamp_problem = problem_fn(n_blocks=1, n_goals=1, n_robots=1)
    print(tamp_problem)

    stream_info = {
        #'test-region': StreamInfo(eager=True, p_success=0), # bound_fn is None
        #'plan-motion': StreamInfo(p_success=1),  # bound_fn is None
        #'trajcollision': StreamInfo(p_success=1),  # bound_fn is None
        #'cfree': StreamInfo(eager=True),
    }

    pddlstream_problem = pddlstream_from_tamp(tamp_problem)
    with Profiler():
        solution = solve_incremental(pddlstream_problem, complexity_step=1, max_time=30,
                                     unit_costs=unit_costs, verbose=False)
        print_solution(solution)
        plan, cost, evaluations = solution
    if plan is None:
        return

    colors = dict(zip(sorted(tamp_problem.initial.block_poses.keys()), COLORS))
    viewer = ContinuousTMPViewer(SUCTION_HEIGHT, tamp_problem.regions, title='Continuous TAMP')
    state = tamp_problem.initial
    print()
    print(state)
    draw_state(viewer, state, colors)
    for i, (action, args) in enumerate(plan):
        user_input('Continue?')
        print(i, action, args)
        state = args[-1]
        print(state)
        draw_state(viewer, state, colors)
    user_input('Finish?')


if __name__ == '__main__':
    main()
