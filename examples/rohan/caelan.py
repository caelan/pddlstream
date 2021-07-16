import time
import argparse
from pddlstream.language.stream import StreamInfo
from pddlstream.language.generator import from_gen_fn
from pddlstream.algorithms.focused import solve_focused
from pddlstream.algorithms.incremental import solve_incremental
from pddlstream.algorithms.meta import solve
from pddlstream.language.constants import PDDLProblem, And, print_solution
from pddlstream.utils import Profiler


def sampler(*args):
    while True:
        yield tuple(0 for val in args[1:])


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--num_attrs", type=int, required=True)
    args = parser.parse_args()
    num_attrs = args.num_attrs
    start = time.time()
    #attr_types = [f"(IsAttr{i} ?robot ?attr{i})" for i in range(num_attrs)]
    attr_types = [f"(IsAttr{i} ?attr{i})" for i in range(num_attrs)]
    attr_types1 = "\n    ".join(attr_types)
    attr_types2 = "\n        ".join(attr_types)
    attr_types3 = " ".join(attr_types)
    #new_attr_types = [f"(IsAttr{i} ?robot ?new-attr{i})" for i in range(num_attrs)]
    new_attr_types = [f"(IsAttr{i} ?new-attr{i})" for i in range(num_attrs)]
    new_attr_types1 = "\n        ".join(new_attr_types)
    new_attr_types2 = " ".join(new_attr_types)
    constraint_params = " ".join([f"?attr{i}" for i in range(num_attrs)]+[f"?new-attr{i}" for i in range(num_attrs)])
    operator_params = " ".join(["?robot", "?task"]+[f"?attr{i}" for i in range(num_attrs)]+[f"?new-attr{i}" for i in range(num_attrs)])
    operator_effects = []
    for i in range(num_attrs):
        #operator_effects.append(f"(not (IsAttr{i} ?robot ?attr{i}))")
        #operator_effects.append(f"(IsAttr{i} ?robot ?new-attr{i})")
        operator_effects.append(f"(not (IsAttr{i} ?attr{i}))")
        operator_effects.append(f"(IsAttr{i} ?new-attr{i})")
    # operator_effects = "\n        ".join(operator_effects)
    stream_inputs = " ".join(["?robot"]+[f"?attr{i}" for i in range(num_attrs)])
    stream_outputs = " ".join([f"?new-attr{i}" for i in range(num_attrs)])
    domain_pddl = f"""(define (domain caelan)
  (:requirements :strips)
  (:predicates
    ; Types
    (IsRobot ?robot)
    (IsTask ?task)
    {attr_types1}

    ; Constraints
    (constraint {constraint_params})

    ; Goal fluents
    (Completed ?r ?t)
  )

  (:action do
    :parameters ({operator_params})
    :precondition (and
        ; Type preconditions
        (IsRobot ?robot)
        (IsTask ?task)
        {attr_types2}
        {new_attr_types1}

        ; Constraint precondition
        (constraint {constraint_params})
    )
    :effect (and
        ; Goal fluents
        (Completed ?robot ?task)
    )
  )
)"""
    print("Domain:")
    print(domain_pddl)
    stream_pddl = f"""(define (stream caelan)
(:stream constraint-sampler
    :inputs ({stream_inputs})
    :domain (and (IsRobot ?robot) {attr_types3})
    :outputs ({stream_outputs})
    :certified (and {new_attr_types2} (constraint {constraint_params}))
    )
)"""
    print("Stream:")
    print(stream_pddl)
    stream_map = {"constraint-sampler": from_gen_fn(sampler)}
    constant_map = {}
    init = [("IsRobot", "robby")]
    num_tasks = 10
    for i in range(num_tasks):
        init.append(("IsTask", f"task{i}"))
    for i in range(num_attrs):
        #init.append((f"IsAttr{i}", "robby", float(i+1)))
        init.append((f"IsAttr{i}", float(i+1)))
    goal = And(*[("Completed", "robby", f"task{i}") for i in range(num_tasks)])
    print("Init state:")
    print(init)
    print("Goal:")
    print(goal)
    print("Running pddlstream:")
    pddlstream_problem = PDDLProblem(
        domain_pddl, constant_map, stream_pddl,
        stream_map, init, goal)
    with Profiler(field='cumtime', num=25):
        solution = solve(pddlstream_problem, algorithm='incremental',
                         max_skeletons=None,
                         planner="ff-eager", unit_costs=True,
                         unit_efforts=True, effort_weight=1, debug=True)
    print_solution(solution)
    print(f"Total time: {time.time()-start:.5f} seconds")


if __name__ == "__main__":
    main()
