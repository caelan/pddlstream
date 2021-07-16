from pddlstream.algorithms.focused import solve_focused
from pddlstream.algorithms.incremental import solve_incremental
from pddlstream.language.generator import from_gen_fn, from_fn, from_test
from pddlstream.language.constants import And, Exists, PDDLProblem, print_solution
from pddlstream.language.stream import StreamInfo
from pddlstream.utils import read, get_file_path

import numpy as np


def get_next_state(state, action):
    return (state + action,)

def get_action_gen(choices):
    def gen_fn():
        while True:
            a = np.random.choice(choices)
            yield (a,)
    return gen_fn

def check_goal_gen():
    def gen_fn(s):
        return s == 10
    return gen_fn

def create_problem():
    domain_pddl = read(get_file_path(__file__, 'domain.pddl'))
    stream_pddl = read(get_file_path(__file__, 'stream.pddl'))
    constant_map = {}
    stream_map = {
        'physics' : from_fn(get_next_state),
        'positive-actions' : from_gen_fn(get_action_gen([3, 2, 1])),
        'check-goal' : from_test(check_goal_gen()),
    }

    init = [
        ('CurrentState', 0),
        ('State', 0),
    ]
    goal = Exists(['?s'], And(('CurrentState', '?s'), ('IsGoalState', '?s')))
    goal = ('AtGoal',)

    return PDDLProblem(domain_pddl, constant_map, stream_pddl, stream_map, init, goal)

def main():
    import numpy as np
    np.random.seed(0)

    # Create PDDLStream problem
    problem = create_problem()
    print('Initial:', problem.init)
    print('Goal:', problem.goal)

    # Not sure what this does...
    stream_info = {
        'physics': StreamInfo(eager=False),
        'positive-actions': StreamInfo(eager=False),
        'check-goal': StreamInfo(eager=False),
    }

    # Solve
    # solution = solve_incremental(problem)
    solution = solve_focused(problem, max_skeletons=None, bind=False, unit_costs=True, stream_info=stream_info)
    print_solution(solution)


if __name__ == '__main__':
    main()
