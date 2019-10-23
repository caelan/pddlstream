#!/usr/bin/env python

from __future__ import print_function

import argparse

from examples.continuous_tamp.primitives import get_pose_gen, distance_fn, inverse_kin, \
    get_region_test, plan_motion, MOVE_COST
from examples.continuous_tamp.run import display_plan, initialize, create_problem, dump_pddlstream
from examples.continuous_tamp.unfactored.run import step_plan
from pddlstream.algorithms.incremental import solve_incremental
from pddlstream.language.constants import PDDLProblem, print_solution
from pddlstream.language.function import FunctionInfo
from pddlstream.language.generator import from_gen_fn, from_test, from_fn, from_list_fn
from pddlstream.language.stream import StreamInfo, WildOutput
from pddlstream.language.temporal import retime_plan
from pddlstream.utils import read, INF, get_file_path, Profiler

def test_cfree(r, b1, g1, b2, g2):
    return True

def get_connect(initial_state):
    vertices = list(initial_state.robot_confs.values())

    def fn(q2):
        facts = []
        for q1 in vertices:
            (t,) = plan_motion(q1, q2)
            facts.extend([
                ('Motion', q1, t, q2),
                ('Traj', t),
            ])
        vertices.append(q2)
        return WildOutput(facts=facts)
    return fn

def pddlstream_from_tamp(tamp_problem):
    domain_pddl = read(get_file_path(__file__, 'domain.pddl'))
    stream_pddl = read(get_file_path(__file__, 'stream.pddl'))

    constant_map = {}
    stream_map = {
        'connect': from_list_fn(get_connect(tamp_problem.initial)),
        't-cfree': from_test(test_cfree),
        's-region': from_gen_fn(get_pose_gen(tamp_problem.regions)),
        't-region': from_test(get_region_test(tamp_problem.regions)),
        's-ik': from_fn(lambda b, p, g: (inverse_kin(p, g),)),
        'dist': distance_fn,
    }
    init, goal = create_problem(tamp_problem)

    return PDDLProblem(domain_pddl, constant_map, stream_pddl, stream_map, init, goal)

def main():
    parser = argparse.ArgumentParser()

    tamp_problem, args = initialize(parser)
    pddlstream_problem = pddlstream_from_tamp(tamp_problem)
    dump_pddlstream(pddlstream_problem)

    with Profiler():
        solution = solve_incremental(pddlstream_problem, planner='ff-wastar1', max_time=args.max_time, verbose=False)
        print_solution(solution)
    plan, cost, evaluations = solution
    step_plan(tamp_problem, plan)
    #if plan is not None:
    #    display_plan(tamp_problem, retime_plan(plan))

if __name__ == '__main__':
    main()
