import numpy as np

from pddlstream.algorithms.focused import solve_focused
from pddlstream.language.generator import from_gen_fn, from_fn
from pddlstream.language.constants import PDDLProblem, print_solution
from pddlstream.language.stream import StreamInfo
from pddlstream.utils import read, get_file_path


def get_next_state(state, action):
    return (state + action,)

def get_action_gen(choices):
    def gen_fn():
        while True:
            a = np.random.choice(choices)
            yield (a,)
    return gen_fn

def create_problem():
    domain_pddl = read(get_file_path(__file__, 'domain.pddl'))
    external_pddl = [read(get_file_path(__file__, 'stream.pddl'))]
    constant_map = {}
    stream_map = {
        'physics': from_fn(get_next_state),
        'positive-actions' : from_gen_fn(get_action_gen([1, 2, 3])),
        'negative-actions' : from_gen_fn(get_action_gen([-1, -2, -3])),
    }
    init = [
        ('CurrentState', 0.),
        ('State', 0.),
    ]
    goal = ('CurrentState', 10.0)

    return PDDLProblem(domain_pddl, constant_map, external_pddl, stream_map, init, goal)

def main():
    # Create PDDLStream problem
    problem = create_problem()
    print('Initial:', problem.init)
    print('Goal:', problem.goal)

    # Not sure what this does...
    stream_info = {
        'physics': StreamInfo(eager=True, p_success=1),
        'positive-actions': StreamInfo(eager=True, p_success=1),
        'negative-actions': StreamInfo(eager=True, p_success=1),
    }

    # Solve
    # solution = solve_focused(problem, stream_info=stream_info, planner='max-astar',
    #                          verbose=True)
    solution = solve_focused(problem, stream_info=stream_info, verbose=True)

    print_solution(solution)


if __name__ == '__main__':
    main()
