from pddlstream.language.constants import And
from pddlstream.language.stream import DEBUG
from pddlstream.algorithms.focused import solve_focused
from pddlstream.language.constants import Equal, AND, PDDLProblem
from pddlstream.utils import print_solution, read, INF, get_file_path, find_unique

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
        #('AtPose', COASTER, block_goal),
        #('On', CUP, COASTER),
        ('HasCoffee', CUP),
        ('HasCream', CUP),
        #('HasSugar', CUP),
        #('Mixed', CUP),
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
    solution = solve_focused(problem, unit_costs=True, planner='ff-eager', debug=True)
    print_solution(solution)

if __name__ == '__main__':
    main()