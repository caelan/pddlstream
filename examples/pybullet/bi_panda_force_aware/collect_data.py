#!/usr/bin/env python

from __future__ import print_function

from examples.pybullet.bi_panda_force_aware.streams import get_cfree_approach_pose_test, get_cfree_pose_pose_test, get_cfree_traj_pose_test, \
    get_cfree_traj_grasp_pose_test, distance_fn

from examples.pybullet.utils.pybullet_tools.panda_primitives_v2 import Pose, Conf, get_ik_ir_gen, \
    get_stable_gen, get_grasp_gen, control_commands, get_torque_limits_not_exceded_test, get_sample_stable_holding_conf_gen, \
    get_stable_gen_dumb, get_torque_limits_mock_test, get_ik_ir_gen_no_reconfig, get_objects_on_target
from examples.pybullet.utils.pybullet_tools.panda_utils import get_arm_joints, ARM_NAMES, get_group_joints, \
    get_group_conf, get_group_links, BI_PANDA_GROUPS, arm_from_arm, TARGET
from examples.pybullet.utils.pybullet_tools.utils import connect, get_pose, is_placement, disconnect, \
    get_joint_positions, HideOutput, LockRenderer, wait_for_user, get_max_limit, set_joint_positions_torque, set_point, get_mass
from examples.pybullet.namo.stream import get_custom_limits

from pddlstream.algorithms.meta import create_parser, solve
from pddlstream.algorithms.common import SOLUTIONS
from pddlstream.language.generator import from_gen_fn, from_list_fn, from_fn, from_test
from pddlstream.language.constants import Equal, And, print_solution, Exists, get_args, is_parameter, \
    get_parameter_name, PDDLProblem
from pddlstream.utils import read, INF, get_file_path, Profiler
from pddlstream.language.function import FunctionInfo
from pddlstream.language.stream import StreamInfo, DEBUG

from examples.pybullet.utils.pybullet_tools.panda_primitives_v2 import apply_commands, State
from examples.pybullet.utils.pybullet_tools.utils import draw_base_limits, WorldSaver, has_gui, str_from_object, joint_from_name

from examples.pybullet.bi_panda.problems import PROBLEMS
from examples.pybullet.utils.pybullet_tools.panda_primitives_v2 import Pose, Conf, get_ik_ir_gen, get_motion_gen, \
    get_stable_gen, get_grasp_gen, Attach, Detach, Clean, Cook, control_commands, \
    get_gripper_joints, GripperCommand, apply_commands, State
import time
import datetime
import csv
import pybullet as p

# TODO: collapse similar streams into a single stream when reodering

def get_bodies_from_type(problem):
    bodies_from_type = {}
    for body, ty in problem.body_types:
        bodies_from_type.setdefault(ty, set()).add(body)
    return bodies_from_type

def pddlstream_from_problem(problem, base_limits=None, collisions=True, teleport=False, name = None):
    robot = problem.robot

    domain_pddl = read(get_file_path(__file__, 'domain.pddl'))
    stream_pddl = read(get_file_path(__file__, 'stream.pddl'))
    constant_map = {
        '@sink': 'sink',
        '@stove': 'stove',
    }

    # initial_bq = Pose(robot, get_pose(robot))
    initial_bq = Conf(robot, get_group_joints(robot, 'base'), get_group_conf(robot, 'base'))
    init = [
        ('BConf', initial_bq),
        ('AtBConf', initial_bq),
        Equal(('PickCost',), 1),
        Equal(('PlaceCost',), 1),
        Equal(('ReconfigureCost',), 1),
    ] + [('Sink', s) for s in problem.sinks] + \
           [('Stove', s) for s in problem.stoves] + \
           [('Connected', b, d) for b, d in problem.buttons] + \
           [('Button', b) for b, _ in problem.buttons]
    for arm in ARM_NAMES:
    #for arm in problem.arms:
        joints = get_arm_joints(robot, arm)
        conf = Conf(robot, joints, get_joint_positions(robot, joints))
        init += [('Arm', arm), ('AConf', arm, conf), ('HandEmpty', arm), ('AtAConf', arm, conf), ('TorqueLimitsNotExceded', arm)]
        if arm in problem.arms:
            init += [('Controllable', arm)]
    for body in problem.movable:
        pose = Pose(body, get_pose(body), init=True) # TODO: supported here
        init += [('Graspable', body), ('Pose', body, pose),
                 ('AtPose', body, pose), ('Stackable', body, None)]
        for surface in problem.surfaces:
            if is_placement(body, surface):
                init += [('Supported', body, pose, surface)]
    for body, ty in problem.body_types:
        init += [('Type', body, ty)]

    bodies_from_type = get_bodies_from_type(problem)
    goal_literals = []
    # if problem.goal_conf is not None:
    #     goal_conf = Conf(robot, get_group_joints(robot, 'base'), problem.goal_conf)
    #     init += [('BConf', goal_conf)]
    #     goal_literals += [('AtBConf', goal_conf)]
    for ty, s in problem.goal_on:
        bodies = bodies_from_type[get_parameter_name(ty)] if is_parameter(ty) else [ty]
        init += [('Stackable', b, s) for b in bodies]
        goal_literals += [('On', ty, s)]
    goal_literals += [('Holding', a, b) for a, b in problem.goal_holding] + \
                     [('Cleaned', b)  for b in problem.goal_cleaned] + \
                     [('Cooked', b)  for b in problem.goal_cooked] + \
                    [('TorqueLimitsNotExceded', a) for a in problem.arms]
    goal_formula = []
    for literal in goal_literals:
        parameters = [a for a in get_args(literal) if is_parameter(a)]
        if parameters:
            type_literals = [('Type', p, get_parameter_name(p)) for p in parameters]
            goal_formula.append(Exists(parameters, And(literal, *type_literals)))
        else:
            goal_formula.append(literal)
    goal_formula = And(*goal_formula)

    custom_limits = {}
    if base_limits is not None:
        custom_limits.update(get_custom_limits(robot, problem.base_limits))



    stream_map = {
        'sample-pose': from_gen_fn(get_stable_gen(problem, collisions=collisions)),
        'sample-grasp': from_list_fn(get_grasp_gen(problem, collisions=collisions)),
        'test-cfree-pose-pose': from_test(get_cfree_pose_pose_test(collisions=collisions)),
        'test-cfree-approach-pose': from_test(get_cfree_approach_pose_test(problem, collisions=collisions)),
        'test-cfree-traj-pose': from_test(get_cfree_traj_pose_test(robot, collisions=collisions))
    }
    if (name == 'bi_manual_forceful' or name == 'bi_manual_forceful_bin'):
        stream_map['sample-pose'] =  from_gen_fn(get_stable_gen(problem, collisions=collisions))
        stream_map["inverse-kinematics"] = from_gen_fn(get_ik_ir_gen(problem, custom_limits=custom_limits,
                                                            collisions=collisions, teleport=teleport))
        stream_map['test_torque_limits_not_exceded'] = from_test(get_torque_limits_not_exceded_test(problem))
    else:
        stream_map['sample-pose'] =  from_gen_fn(get_stable_gen_dumb(problem, collisions=collisions))
        stream_map["inverse-kinematics"] = from_gen_fn(get_ik_ir_gen_no_reconfig(problem, custom_limits=custom_limits,
                                                            collisions=collisions, teleport=teleport))
        stream_map['test_torque_limits_not_exceded'] = from_test(get_torque_limits_mock_test(problem))
    #stream_map = DEBUG

    return PDDLProblem(domain_pddl, constant_map, stream_pddl, stream_map, init, goal_formula)

#######################################################
def post_process(problem, plan, teleport=False):
    if plan is None:
        return None
    commands = []
    for i, (name, args) in enumerate(plan):
        if name == 'move_base':
            c = args[-1]
            new_commands = c.commands
        elif name == 'pick':
            a, b, p, g, _, c = args
            trajs = c.commands
            close_gripper = GripperCommand(problem.robot, a, g.grasp_width, teleport=teleport)
            attach = Attach(problem.robot, a, g, b)
            if len(trajs) == 2:
                print("reconfig present")
                [t1, t2] = trajs
                new_commands = [t1, t2, close_gripper, attach, t2.reverse()]
            else:
                print("no reconfig")
                [t2] = trajs
                new_commands = [t2, close_gripper, attach, t2.reverse()]
        elif name == 'place':
            a, b, p, g, _, c, _ = args
            trajectories = c.commands
            gripper_joint = get_gripper_joints(problem.robot, a)[0]
            position = 0.05
            open_gripper = GripperCommand(problem.robot, a, position, teleport=teleport)
            detach = Detach(problem.robot, a, b)
            if len(trajectories) == 2:
                print("reconfig present")
                [t1, t2] = c.commands
                new_commands = [t1, t2, detach, open_gripper, t2.reverse()]
            else:
                print("no reconfig")
                [t2] = c.commands
                new_commands = [t2, detach, open_gripper, t2.reverse()]
        elif name == 'clean': # TODO: add text or change color?
            body, sink = args
            new_commands = [Clean(body)]
        elif name == 'cook':
            body, stove = args
            new_commands = [Cook(body)]
        elif name == 'press_clean':
            body, sink, arm, button, bq, c = args
            [t] = c.commands
            new_commands = [t, Clean(body), t.reverse()]
        elif name == 'press_cook':
            body, sink, arm, button, bq, c = args
            [t] = c.commands
            new_commands = [t, Cook(body), t.reverse()]
        else:
            raise ValueError(name)
        print(i, name, args, new_commands)
        commands += new_commands
    return commands

data_dir = '/home/liam/exp_data/'

def main(verbose=True):
    # TODO: could work just on postprocessing
    # TODO: try the other reachability database
    # TODO: option to only consider costs during local optimization


    parser = create_parser()
    parser.add_argument('-problem', default='bi_manual_forceful', help='The name of the problem to solve')
    parser.add_argument('-loops', default=1, type=int, help='The number of itterations to run experiment')
    parser.add_argument('-n', '--number', default=5, type=int, help='The number of objects')
    parser.add_argument('-cfree', action='store_true', help='Disables collisions')
    parser.add_argument('-deterministic', action='store_true', help='Uses a deterministic sampler')
    parser.add_argument('-optimal', action='store_true', help='Runs in an anytime mode')
    parser.add_argument('-t', '--max_time', default=250, type=int, help='The max time')
    parser.add_argument('-teleport', action='store_true', help='Teleports between configurations')
    parser.add_argument('-enable', action='store_true', help='Enables rendering during planning')
    parser.add_argument('-simulate', action='store_true', help='Simulates the system')
    args = parser.parse_args()
    print('Arguments:', args)


    timestamp = str(datetime.datetime.now())
    timestamp = "{}_{}".format(timestamp.split(' ')[0], timestamp.split(' ')[1])
    datafile = data_dir + timestamp + "_" + args.problem + '.csv'
    data = ["TotalTime", "ExecutionTime", "Solved", "TotalItems", "SuccessfulDeliveries", "Mass per Object"]
    with open(datafile, 'w') as file:
        writer = csv.writer(file)
        writer.writerow(data)


    problem_fn_from_name = {fn.__name__: fn for fn in PROBLEMS}
    if args.problem not in problem_fn_from_name:
        raise ValueError(args.problem)
    if args.problem == "bi_manual_forceful_bin":
        global TARGET
        TARGET = "bin"
    problem_fn = problem_fn_from_name[args.problem]
    print('problem fn loaded')
    for _ in range(args.loops):
      connect(use_gui=True)
      print('connected to gui')
      with HideOutput():
          problem = problem_fn(num=args.number)
      print('problem found')
      draw_base_limits(problem.base_limits, color=(1, 0, 0))
      saver = WorldSaver()
      print("world made")
      #handles = []
      #for link in get_group_joints(problem.robot, 'left_arm'):
      #    handles.append(draw_link_name(problem.robot, link))
      #wait_for_user()

      pddlstream_problem = pddlstream_from_problem(problem, collisions=not args.cfree, teleport=False, name=args.problem)
      stream_info = {
          'inverse-kinematics': StreamInfo(),
          'plan-base-motion': StreamInfo(overhead=1e1),
          'test_torque_limits_not_exceded': StreamInfo(p_success=1e-1),
          'test-cfree-pose-pose': StreamInfo(p_success=1e-3, verbose=verbose),
          'test-cfree-approach-pose': StreamInfo(p_success=1e-2, verbose=verbose),
          'test-cfree-traj-pose': StreamInfo(p_success=1e-1, verbose=verbose), # TODO: apply to arm and base trajs
          # 'test-forces-balanced': StreamInfo(p_success=1e-1, verbose=verbose),
          #'test-cfree-traj-grasp-pose': StreamInfo(verbose=verbose),

          # 'Distance': FunctionInfo(p_success=0.99, opt_fn=lambda q1, q2: 0),

          #'MoveCost': FunctionInfo(lambda t: BASE_CONSTANT),
      }
      #stream_info = {}

      _, _, _, stream_map, init, goal = pddlstream_problem
      print('Init:', init)
      print('Goal:', goal)
      print('Streams:', str_from_object(set(stream_map)))
      jointNums = []
      joints = BI_PANDA_GROUPS[arm_from_arm('left')]
      for joint in joints:
          jointNums.append(joint_from_name(problem.robot, joint))
      success_cost = 0 if args.optimal else INF
      planner = 'ff-astar' if args.optimal else 'ff-wastar3'
      search_sample_ratio = 2
      max_planner_time = 20
      # effort_weight = 0 if args.optimal else 1
      effort_weight = 1e-3 if args.optimal else 1
      start_time = time.time()
      with Profiler(field='tottime', num=25): # cumtime | tottime
          with LockRenderer(lock=not args.enable):
              solution = solve(pddlstream_problem, algorithm=args.algorithm, stream_info=stream_info,
                              planner=planner, max_planner_time=max_planner_time,
                              unit_costs=args.unit, success_cost=success_cost,
                              max_time=args.max_time, verbose=True, debug=True,
                              unit_efforts=True, effort_weight=effort_weight,
                              search_sample_ratio=search_sample_ratio)
              saver.restore()


      cost_over_time = [(s.cost, s.time) for s in SOLUTIONS]
      for i, (cost, runtime) in enumerate(cost_over_time):
          print('Plan: {} | Cost: {:.3f} | Time: {:.3f}'.format(i, cost, runtime))
      #print(SOLUTIONS)
      print_solution(solution)
      plan, cost, evaluations = solution
      if (plan is None) or not has_gui():
        total_time = time.time() - start_time
        exec_time = -1
        items = args.number
        deliveries = 0
        solved = False

        print(total_time, exec_time, solved, items, deliveries)
        data = [total_time, exec_time, solved, items, deliveries]
        with open(datafile, 'a') as file:
            writer = csv.writer(file)
            writer.writerow(data)
            disconnect()
        continue

      with LockRenderer(lock=not args.enable):
          problem.remove_gripper()
          commands = post_process(problem, plan, teleport=args.teleport)
          saver.restore()
      p.setRealTimeSimulation(True)

      # jointPos = get_joint_positions(problem.robot, jointNums)
      # set_joint_positions_torque(problem.robot, jointNums, jointPos)
      draw_base_limits(problem.base_limits, color=(1, 0, 0))
      jointPos = get_joint_positions(problem.robot, jointNums)
      set_joint_positions_torque(problem.robot, jointNums, jointPos)
      exec_time = time.time()
      if args.simulate:
          control_commands(commands)
      else:
          time_step = None if args.teleport else 0.01
          apply_commands(State(), commands, time_step)

      jointPos = get_joint_positions(problem.robot, jointNums)
      set_joint_positions_torque(problem.robot, jointNums, jointPos)

      total_time = time.time() - start_time
      exec_time = time.time() - exec_time
      items = args.number
      deliveries = len(get_objects_on_target(problem))
      solved = True
      mass = get_mass(problem.movable[0])

      print(total_time, exec_time, items, deliveries, mass)
      data = [total_time, exec_time, solved, items, deliveries, mass]
      with open(datafile, 'a') as file:
        writer = csv.writer(file)
        writer.writerow(data)
      p.setRealTimeSimulation(True)
      disconnect()

if __name__ == '__main__':
    main()