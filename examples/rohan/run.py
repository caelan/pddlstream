"""PDDLStream experiment runfile.
"""

import time
import functools
import random
import numpy as np
from pddlstream.algorithms.algorithm import reset_globals
from pddlstream.algorithms.incremental import solve_incremental
from pddlstream.algorithms.focused import solve_focused
from pddlstream.language.stream import StreamInfo
from pddlstream.language.generator import from_gen_fn
from pddlstream.language.constants import PDDLProblem, print_solution, And
from pddlstream.utils import Profiler

NUM_OBJECTS = 3
NUM_TABLES = 500 # 100 | 250 | 500

def _run(planner):
    np.random.seed(0)
    random.seed(0)
    rng = random
    reset_globals()

    operators = []
    for i in range(NUM_TABLES):
        operator = f"""(:action placeontable{i}
    :parameters (?obj ?cur-pose ?new-pose ?conf)
    :precondition (and
        ; Types
        (IsObject ?obj)
        (IsPose ?cur-pose)
        (IsPose ?new-pose)
        (IsConf ?conf)
        ; Controller
        (Place ?conf)
        ; Read-off fluents
        (HasPose ?obj ?cur-pose)
        ; Constraint
        (Constraint{i} ?cur-pose ?new-pose ?conf)
    )
    :effect (and
        ; Read-off fluents
        (not (HasPose ?obj ?cur-pose))
        (HasPose ?obj ?new-pose)
        ; Goal fluents
        (OnTable{i} ?obj)
    )
  )
"""
        operators.append(operator)
    operators = "\n    ".join(operators)

    constraints = "\n    ".join([f"(Constraint{i} ?cp ?np ?c)" \
                                 for i in range(NUM_TABLES)])
    goal_fluents = "\n    ".join([f"(OnTable{i} ?o)" \
                                  for i in range(NUM_TABLES)])

    domain_pddl = f"""
(define (domain buttons)
  (:requirements :strips)
  (:predicates
    ; Types
    (IsObject ?o)
    (IsPose ?p)
    (IsConf ?c)

    ; Controllers
    (Place ?c)

    ; Read-off fluents
    (HasPose ?o ?p)

    ; Constraints
    {constraints}

    ; Goal fluents
    {goal_fluents}
  )

  {operators}

)

"""

    stream_strs = []
    for i in range(NUM_TABLES):
        stream_str = f"""(:stream constraint{i}-sampler
    :inputs (?cur-pose)
    :domain (and (IsPose ?cur-pose))
    :outputs (?new-pose ?conf)
    :certified (and
        (IsPose ?new-pose)
        (IsConf ?conf)
        (Place ?conf)
        (Constraint{i} ?cur-pose ?new-pose ?conf))
    )"""
        stream_strs.append(stream_str)

    stream_strs = "\n    ".join(stream_strs)
    stream_pddl = f"""(define (stream buttons)
    {stream_strs}
)
"""

    def constraint_sampler(table, cur_pose):
        if cur_pose == -1.0:
            new_pose = rng.uniform(table, table+1)
            yield (new_pose, new_pose+100)  # conf = pose+100

    stream_map = {
        f"constraint{i}-sampler": from_gen_fn(
            functools.partial(constraint_sampler, i)) \
        for i in range(NUM_TABLES)
    }

    init = []
    init.append(("IsPose", -1.0))  # init pose of all objects
    init.append(("IsConf", -100.0))  # init conf
    for i in range(NUM_OBJECTS):
        init.append(("IsObject", f"object{i}"))
        init.append(("HasPose", f"object{i}", -1.0))

    constant_map = {}

    goal = And(*[(f"OnTable{NUM_TABLES-1}", f"object{i}")
                 for i in range(NUM_OBJECTS)])

    pddl_problem = PDDLProblem(domain_pddl, constant_map, stream_pddl,
                               stream_map, init, goal)

    with Profiler():
        if planner == "focused":
            stream_info = {k: StreamInfo(eager=False, p_success=1)
                           for k in stream_map}
            solution = solve_focused(pddl_problem, stream_info=stream_info,
                                     max_skeletons=None, max_time=1000, debug=True)
        elif planner == "incremental":
            solution = solve_incremental(pddl_problem, max_time=1000, debug=True)
    print_solution(solution)



if __name__ == "__main__":
    start_time = time.time()
    #_run(planner="incremental")
    _run(planner="focused")
    print("Universe time:", time.time() - start_time)
