#!/usr/bin/env python

from __future__ import print_function

try:
    import pybullet as p
except ImportError:
    raise ImportError('This example requires PyBullet (https://pypi.org/project/pybullet/)')

import cProfile
import pstats

from pddlstream.focused import solve_focused
from pddlstream.stream import from_fn, from_gen_fn, from_list_fn
from pddlstream.utils import print_solution, read, get_file_path
from pddlstream.conversion import Equal, Problem, And

from examples.pybullet.pr2_belief.primitives import Scan, Detect, get_vis_gen, Register, plan_head_traj
from examples.pybullet.pr2_belief.problems import get_problem1
from examples.pybullet.utils.pybullet_tools.pr2_utils import DRAKE_PR2_URDF, ARM_NAMES, get_arm_joints
from examples.pybullet.utils.pybullet_tools.utils import set_pose, get_pose, load_model, connect, clone_world, \
    disconnect, set_client, add_data_path, WorldSaver, wait_for_interrupt, get_joint_positions, get_configuration, \
    set_configuration, ClientSaver, HideOutput, is_center_stable, add_body_name
from examples.pybullet.utils.pybullet_tools.pr2_primitives import Conf, get_ik_ir_gen, get_motion_gen, get_stable_gen, \
    get_grasp_gen, Attach, Detach, apply_commands
from examples.discrete_belief.run import scale_cost, revisit_mdp_cost, SCALE_COST, MAX_COST


def pddlstream_from_state(state, teleport=False):
    task = state.task
    robot = task.robot
    # TODO: infer open world from task

    domain_pddl = read(get_file_path(__file__, 'domain.pddl'))
    stream_pddl = read(get_file_path(__file__, 'stream.pddl'))
    constant_map = {
        'base': 'base',
        'left': 'left',
        'right': 'right',
        'head': 'head',
    }

    scan_cost = 1
    init = [
        ('BConf', state.poses[robot]),
        ('AtBConf', state.poses[robot]),
        Equal(('MoveCost',), scale_cost(1)),
        Equal(('PickCost',), scale_cost(1)),
        Equal(('PlaceCost',), scale_cost(1)),
        Equal(('ScanCost',), scale_cost(scan_cost)),
        Equal(('RegisterCost',), scale_cost(1)),
    ]
    holding_arms = set()
    holding_bodies = set()
    for attach in state.attachments.values():
        holding_arms.add(attach.arm)
        holding_bodies.add(attach.body)
        init += [('Grasp', attach.body, attach.grasp),
                 ('AtGrasp', attach.arm, attach.body, attach.grasp)]
    for arm in ARM_NAMES:
       joints = get_arm_joints(robot, arm)
       conf = Conf(robot, joints, get_joint_positions(robot, joints))
       init += [('Arm', arm), ('AConf', arm, conf), ('AtAConf', arm, conf)]
       if arm in task.arms:
           init += [('Controllable', arm)]
       if arm not in holding_arms:
           init += [('HandEmpty', arm)]
    for body in task.movable + task.surfaces:
        if body in holding_bodies:
            continue
        pose = state.poses[body]
        init += [('Pose', body, pose), ('AtPose', body, pose),
                 ('Observable', pose),
        ]

    for body in task.movable:
        init += [('Graspable', body)]
        for surface in task.get_supports(body):
            p_obs = state.b_on[body].prob(surface)
            cost = revisit_mdp_cost(0, scan_cost, p_obs)  # TODO: imperfect observation model
            init += [('Stackable', body, surface),
                     Equal(('LocalizeCost', surface, body), scale_cost(cost))]
            #if is_placement(body, surface):
            if is_center_stable(body, surface):
                if body in holding_bodies:
                    continue
                pose = state.poses[body]
                init += [('Supported', body, pose, surface)]
    for body in (task.movable + task.surfaces):
        if state.is_localized(body):
            init.append(('Localized', body))
        else:
            init.append(('Uncertain', body))
        if body in state.registered:
            init.append(('Registered', body))

    goal = And(*[('Holding', a, b) for a, b in task.goal_holding] + \
           [('On', b, s) for b, s in task.goal_on] + \
           [('Localized', b) for b in task.goal_localized] + \
           [('Registered', b) for b in task.goal_registered])

    stream_map = {
        'sample-pose': get_stable_gen(task),
        'sample-grasp': from_list_fn(get_grasp_gen(task)),
        'inverse-kinematics': from_gen_fn(get_ik_ir_gen(task, teleport=teleport)),
        'inverse-visibility': from_gen_fn(get_vis_gen(task)),
        'plan-base-motion': from_fn(get_motion_gen(task, teleport=teleport)),
    }

    return Problem(domain_pddl, constant_map, stream_pddl, stream_map, init, goal)


#######################################################

def post_process(problem, plan, replan_obs=True, replan_base=False):
    if plan is None:
        return None
    # TODO: refine actions
    robot = problem.robot
    commands = []
    uncertain_base = False
    expecting_obs = False
    for i, (name, args) in enumerate(plan):
        if replan_obs and expecting_obs:
            break
        if name == 'move_base':
            t = args[-1]
            new_commands = [t]
            if replan_base:
                uncertain_base = True
        elif name == 'pick':
            if uncertain_base:
                break
            a, b, p, g, _, t = args
            attach = Attach(robot, a, g, b)
            new_commands = [t, attach, t.reverse()]
        elif name == 'place':
            if uncertain_base:
                break
            a, b, p, g, _, t = args
            detach = Detach(robot, a, b)
            new_commands = [t, detach, t.reverse()]
        elif name == 'scan':
            o, p, bq, hq = args
            ht = plan_head_traj(robot, hq.values)
            detect = Scan(robot, o)
            new_commands = [ht, detect]
        elif name == 'localize':
            r, _, o, _ = args
            new_commands = [Detect(robot, r, o)]
            expecting_obs = True
        elif name == 'register':
            o, p, bq, hq = args
            ht = plan_head_traj(robot, hq.values)
            register = Register(robot, o)
            new_commands = [ht, register]
            expecting_obs = True
        else:
            raise ValueError(name)
        # TODO: execute these here?
        commands += new_commands
    return commands


#######################################################

def plan_commands(task, state, teleport=False, profile=False, verbose=False):
    # TODO: could make indices into set of bodies to ensure the same...
    # TODO: populate the bodies here from state
    robot_conf = get_configuration(task.robot)
    robot_pose = get_pose(task.robot)
    sim_world = connect(use_gui=False)
    with ClientSaver(sim_world):
        with HideOutput():
            robot = load_model(DRAKE_PR2_URDF, fixed_base=True)
        set_pose(robot, robot_pose)
        set_configuration(robot, robot_conf)
    clone_world(client=sim_world, exclude=[task.robot])
    set_client(sim_world)
    saved_world = WorldSaver() # StateSaver()

    pddlstream_problem = pddlstream_from_state(state, teleport=teleport)
    _, _, _, stream_map, init, goal = pddlstream_problem
    print('Init:', sorted(init, key=lambda f: f[0]))
    if verbose:
        print('Goal:', goal)
        print('Streams:', stream_map.keys())

    pr = cProfile.Profile()
    pr.enable()
    solution = solve_focused(pddlstream_problem, max_cost=MAX_COST, verbose=verbose)
    pr.disable()
    plan, cost, evaluations = solution
    if MAX_COST <= cost:
        plan = None
    print_solution(solution)
    print('Finite cost:', cost < MAX_COST)
    print('Real cost:', float(cost)/SCALE_COST)
    if profile:
        pstats.Stats(pr).sort_stats('tottime').print_stats(10)
    saved_world.restore()
    commands = post_process(task, plan)
    disconnect()
    return commands


#######################################################

def main(time_step=0.01):
    # TODO: closed world and open world
    real_world = connect(use_gui=True)
    add_data_path()
    task, state = get_problem1()
    for body in (task.movable + task.surfaces):
        add_body_name(body)
    #wait_for_interrupt()

    # TODO: automatically determine an action/command cannot be applied
    # TODO: convert numpy arrays into what they are close to
    # TODO: compute whether a plan will still achieve a goal and do that
    # TODO: update the initial state directly and then just redraw it to ensure uniqueness
    step = 0
    while True:
        step += 1
        print('\n' + 50 * '-')
        print(step, state)
        with ClientSaver():
            commands = plan_commands(task, state)
        print()
        if commands is None:
            print('Failure!')
            break
        if not commands:
            print('Success!')
            break
        apply_commands(state, commands, time_step=time_step)

    print(state)
    wait_for_interrupt()
    disconnect()


if __name__ == '__main__':
    main()
