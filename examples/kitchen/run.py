from pddlstream.language.constants import And, print_solution
from pddlstream.language.stream import DEBUG
from pddlstream.algorithms.focused import solve_focused
from pddlstream.language.constants import PDDLProblem
from pddlstream.utils import read, get_file_path

import cProfile
import pstats

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
        ('Clear', COASTER)]

    goal_literals = [
        ('Empty', ROBOT),
        ('AtPose', COASTER, block_goal),
        ('On', CUP, COASTER),
        ('HasCoffee', CUP),
        ('HasCream', CUP),
        ('HasSugar', CUP),
        ('Mixed', CUP),
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
        initial_atoms += [('IsPose', name, pose), ('AtPose', name, pose), ('TableSupport', pose)]

    domain_pddl = read(get_file_path(__file__, 'domain.pddl'))
    stream_pddl = read(get_file_path(__file__, 'stream.pddl'))

    constant_map = {}
    stream_map = DEBUG

    return PDDLProblem(domain_pddl, constant_map, stream_pddl, stream_map,
                       initial_atoms, And(*goal_literals))


def main():
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
    pr = cProfile.Profile()
    pr.enable()
    solution = solve_focused(problem, unit_costs=True, planner='ff-eager', debug=True) # max_planner_time=5,
    pr.disable()
    print_solution(solution)
    pstats.Stats(pr).sort_stats('tottime').print_stats(10)

if __name__ == '__main__':
    main()