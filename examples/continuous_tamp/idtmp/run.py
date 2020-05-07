#!/usr/bin/env python

from __future__ import print_function

import argparse

from examples.continuous_tamp.primitives import get_pose_gen, distance_fn, inverse_kin, \
    get_region_test, plan_motion, MOVE_COST, test_reachable, GRASP
from examples.continuous_tamp.run import display_plan, initialize, create_problem, dump_pddlstream
from examples.continuous_tamp.unfactored.run import step_plan
from pddlstream.algorithms.focused import solve_focused
from pddlstream.algorithms.incremental import solve_incremental
from pddlstream.language.constants import PDDLProblem, print_solution
from pddlstream.language.function import FunctionInfo
from pddlstream.language.generator import from_gen_fn, from_test, from_fn
from pddlstream.language.stream import StreamInfo
from pddlstream.language.temporal import retime_plan
from pddlstream.utils import read, INF, get_file_path, Profiler


def pddlstream_from_tamp(tamp_problem):
    domain_pddl = read(get_file_path(__file__, 'domain.pddl'))
    stream_pddl = read(get_file_path(__file__, 'stream.pddl'))

    # TODO: algorithm that prediscretized once
    constant_map = {}
    stream_map = {
        's-motion': from_fn(plan_motion),
        't-reachable': from_test(test_reachable),
        's-region': from_gen_fn(get_pose_gen(tamp_problem.regions)),
        't-region': from_test(get_region_test(tamp_problem.regions)),
        's-ik': from_fn(lambda b, p, g: (inverse_kin(p, g),)),
        'dist': distance_fn,
    }
    init, goal = create_problem(tamp_problem)
    init.extend(('Grasp', b, GRASP) for b in tamp_problem.initial.block_poses)

    return PDDLProblem(domain_pddl, constant_map, stream_pddl, stream_map, init, goal)

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('-a', '--attachments', action='store_true')
    parser.add_argument('-o', '--optimal', action='store_true', help='Runs in an anytime mode')

    tamp_problem, args = initialize(parser)
    stream_info = {
        't-region': StreamInfo(eager=False, p_success=0),
        'distance': FunctionInfo(opt_fn=lambda q1, q2: MOVE_COST),
    }

    pddlstream_problem = pddlstream_from_tamp(tamp_problem)
    dump_pddlstream(pddlstream_problem)

    success_cost = 0 if args.optimal else INF
    planner = 'max-astar'
    #planner = 'ff-wastar1'
    with Profiler():
        if args.attachments:
            solution = solve_incremental(pddlstream_problem, planner='ff-wastar1',
                                         max_time=args.max_time, verbose=True)
        else:
            solution = solve_focused(pddlstream_problem, stream_info=stream_info,
                                     planner=planner, max_planner_time=10, debug=False,
                                     max_time=args.max_time, max_iterations=INF, verbose=True,
                                     unit_costs=args.unit, success_cost=success_cost,
                                     unit_efforts=False, effort_weight=0,
                                     max_skeletons=None, bind=True,
                                     visualize=args.visualize)

        print_solution(solution)
    plan, cost, evaluations = solution
    step_plan(tamp_problem, plan)
    #if plan is not None:
    #    display_plan(tamp_problem, retime_plan(plan))

if __name__ == '__main__':
    main()
