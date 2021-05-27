from __future__ import print_function

from pddlstream.algorithms import instantiate_task
instantiate_task.FD_INSTANTIATE = True
from pddlstream.algorithms import instantiation
instantiation.USE_RELATION = True
from pddlstream.algorithms import refinement
refinement.CONSTRAIN_PLANS = False

from pddlstream.algorithms.meta import solve, create_parser
from pddlstream.language.constants import And, print_solution
from pddlstream.language.stream import DEBUG, SHARED_DEBUG, StreamInfo, PartialInputs
#from pddlstream.algorithms.serialized import solve_serialized
from pddlstream.language.constants import PDDLProblem, read_pddlstream_pair
from pddlstream.utils import read, get_file_path, Profiler

ROBOT = 'gripper'
CUP = 'cup'
COASTER = 'block'

def create_problem(initial_poses):
    block_goal = (-25, 0, 0)

    initial_atoms = [
        ('IsPose', COASTER, block_goal),
        ('Empty', ROBOT),
        ('CanMove', ROBOT),
        ('HasSugar', 'sugar_cup'),
        ('HasCream', 'cream_cup'),
        ('IsPourable', 'cream_cup'),
        ('Stackable', CUP, COASTER),
        ('Clear', COASTER),
    ]

    goal_literals = [
        ('AtPose', COASTER, block_goal),
        ('On', CUP, COASTER),
        ('HasCoffee', CUP),
        ('HasCream', CUP),
        ('HasSugar', CUP),
        ('Mixed', CUP),
        ('Empty', ROBOT),
    ]

    for name, pose in initial_poses.items():
        if 'gripper' in name:
            initial_atoms += [('IsGripper', name)]
        if 'cup' in name:
            initial_atoms += [('IsCup', name)]
        if 'spoon' in name:
            initial_atoms += [('IsSpoon', name), ('IsStirrer', name)]
        if 'stirrer' in name:
            initial_atoms += [('IsStirrer', name)]
        if 'block' in name:
            initial_atoms += [('IsBlock', name)]
        initial_atoms += [
            ('IsPose', name, pose),
            ('AtPose', name, pose),
            ('TableSupport', pose),
        ]

    domain_pddl, stream_pddl = read_pddlstream_pair(__file__)
    constant_map = {}
    #stream_map = DEBUG
    stream_map = SHARED_DEBUG

    return PDDLProblem(domain_pddl, constant_map, stream_pddl, stream_map,
                       initial_atoms, And(*goal_literals))

# TODO: where did this domain originate?
# https://github.mit.edu/Learning-and-Intelligent-Systems/ltamp_pr2/issues/1
# https://github.mit.edu/caelan/ss/tree/master/examples/kitchen
# https://github.mit.edu/ziw/Kitchen2D
# https://ziw.mit.edu/projects/kitchen2d/
# https://github.com/zi-w/Kitchen2D

def main():
    parser = create_parser()
    args = parser.parse_args()
    print('Arguments:', args)

    initial_poses = {
        ROBOT: (0., 15., 0.),
        CUP: (7.5, 0., 0.),
        'sugar_cup': (-10., 0., 0.),
        'cream_cup': (15., 0, 0),
        'spoon': (0.5, 0.5, 0),
        'stirrer': (20, 0.5, 0),
        COASTER: (-20., 0, 0),
    }

    problem = create_problem(initial_poses)
    stream_info = {
        'sample-grasp-ctrl': StreamInfo(opt_gen_fn=PartialInputs(unique=False)),
    }

    with Profiler(field='tottime'):
        #solution = solve_serialized(problem, planner='ff-eager', unit_costs=args.unit,
        #                            unit_efforts=True, effort_weight=1, debug=False) # max_planner_time=5,
        solution = solve(problem, algorithm=args.algorithm, stream_info=stream_info,
                         unit_costs=args.unit, planner='ff-eager',
                         unit_efforts=True, effort_weight=1, debug=False) # max_planner_time=5,
        print_solution(solution)

if __name__ == '__main__':
    main()