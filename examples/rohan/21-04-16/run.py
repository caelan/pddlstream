"""PDDLStream experiment runfile.

Blocks example. 2D poses.
"""

import time
import random
import numpy as np
from pddlstream.algorithms.algorithm import reset_globals
from pddlstream.algorithms.incremental import solve_incremental
from pddlstream.algorithms.focused import solve_focused
from pddlstream.language.stream import StreamInfo, PartialInputs
from pddlstream.language.generator import from_gen_fn
from pddlstream.language.constants import PDDLProblem, print_solution, And
from pddlstream.utils import read, get_file_path, Profiler


NUM_BLOCKS = 5


def _create_stream_fn(rng, sample_action_fn, ml_next_state_fn,
                      num_objs_in_constraint, pre_attrs, eff_attrs):
    """Helper to create stream functions
    """
    def fn(*args):
        # Args are objects and then state vals
        state_vals = args[num_objs_in_constraint:]
        while True:
            # Sample action
            skill_args = sample_action_fn(rng, state_vals)
            if skill_args is None:
                return  # exhausted rejection sampling attempts
            eff_vals = ml_next_state_fn(state_vals, skill_args)
            yield tuple(skill_args) + tuple(eff_vals)
    return from_gen_fn(fn)


def _pickfromtable_sampler(rng, state_vals):
    # ?block-posex ?block-posey ?block-held ?block-clear
    assert len(state_vals) == 4
    # Can only pick from table if clear and posey is 0
    block_posex, block_posey, _, block_clear = state_vals
    if (not block_clear) or (block_posey != 0):
        return None
    # TODO check handempty (requires change to low level state)
    # Sample pick
    # ?pickparam1, ?pickparam2
    return (block_posex, block_posey)

def _pickfromtable_ns(state_vals, skill_args):
    # ?block-posex ?block-posey ?block-held ?block-clear
    assert len(state_vals) == 4
    # ?pickparam1, ?pickparam2
    assert len(skill_args) == 2
    # -1 is dummy value indicating nowhere
    # ?next-block-posex ?next-block-posey ?next-block-held
    return (-1, -1, 1)

new_place_count = 10000
def _placeontable_sampler(rng, state_vals):
    # ?block-posex ?block-posey ?block-held ?block-clear
    assert len(state_vals) == 4
    # Can only place if held
    if not state_vals[2]:
        return None
    # Sample place
    global new_place_count
    new_place_count += 1
    # ?placeparam1 ?placeparam2
    return (new_place_count, 0)

def _placeontable_ns(state_vals, skill_args):
    assert len(state_vals) == 4
    assert len(skill_args) == 2
    # ?next-block-posex ?next-block-posey ?next-block-held
    place_x, place_y = skill_args
    return (place_x, place_y, 0)

def _unstack_sampler(rng, state_vals):
    # ?block1-posex ?block1-posey ?block1-held ?block1-clear ?block2-posex ?block2-posey ?block2-held ?block2-clear
    assert len(state_vals) == 8
    # Can only unstack if block1 is on block2 and block1 is clear
    block1_x, block1_y = state_vals[0], state_vals[1]
    block2_x, block2_y = state_vals[4], state_vals[5]
    block1_clear = state_vals[3]
    if not (block1_x == block2_x and block1_y == block2_y + 1 and block1_clear):
        return None
    # TODO check handempty (requires change to low level state)
    # ?pickparams1 ?pickparam2
    return (block1_x, block1_y)

def _unstack_ns(state_vals, skill_args):
    assert len(state_vals) == 8
    assert len(skill_args) == 2
    # ?next-block1-posex ?next-block1-posey ?next-block1-held ?next-block2-clear
    return (-1, -1, 1, 1)

def _stack_sampler(rng, state_vals):
    # ?block1-posex ?block1-posey ?block1-held ?block1-clear ?block2-posex ?block2-posey ?block2-held ?block2-clear
    assert len(state_vals) == 8
    # Cannot stack if block1 not held or block2 not clear
    block1_held, block2_clear = state_vals[2], state_vals[7]
    if (not block1_held) or (not block2_clear):
        return None
    block2_posex, block2_posey = state_vals[4], state_vals[5]
    place_x = block2_posex
    place_y = block2_posey + 1
    # ?placeparam1 ?placeparam2
    return (place_x, place_y)

def _stack_ns(state_vals, skill_args):
    # ?block1-posex ?block1-posey ?block1-held ?block1-clear ?block2-posex ?block2-posey ?block2-held ?block2-clear
    assert len(state_vals) == 8
    # ?placeparam1 ?placeparam2
    assert len(skill_args) == 2
    block1_held, block2_clear = state_vals[2], state_vals[7]
    assert block1_held and block2_clear
    # ?next-block1-posex ?next-block1-posey ?next-block1-held ?next-block2-clear
    return (skill_args[0], skill_args[1], 0, 0)

def _run():
    np.random.seed(0)
    random.seed(0)
    rng = random
    reset_globals()

    domain_pddl = read(get_file_path(__file__, "domain.pddl"))
    stream_pddl = read(get_file_path(__file__, "stream.pddl"))

    # block0 is in its own pile
    # block1, ..., blockN are in a second pile with block1 at the bottom
    # goal is always to put block0 on block1
    assert NUM_BLOCKS >= 2
    init = []
    init.append(("HandEmpty",))
    init.append(("IsBlock", "block0"))
    init.append(("HasPoseX", "block0", 0))
    init.append(("HasPoseY", "block0", 0))
    init.append(("HasHeld", "block0", 0))
    init.append(("HasClear", "block0", 1))
    init.append(("AreAttrsForObject", "block0", 0, 0, 0, 1))
    init.append(("OnTable", "block0"))
    init.append(("Clear", "block0"))
    for i in range(1, NUM_BLOCKS):
        init.append(("IsBlock", f"block{i}"))
        init.append(("HasPoseX", f"block{i}", 1))
        init.append(("HasPoseY", f"block{i}", i-1))
        init.append(("HasHeld", f"block{i}", 0))
        if i == NUM_BLOCKS-1:
            init.append(("HasClear", f"block{i}", 1))
            init.append(("AreAttrsForObject", f"block{i}", 1, i-1, 0, 1))
            init.append(("Clear", f"block{i}"))
        else:
            init.append(("HasClear", f"block{i}", 0))
            init.append(("AreAttrsForObject", f"block{i}", 1, i-1, 0, 0))
        if i == 1:
            init.append(("OnTable", f"block{i}"))
        else:
            init.append(("On", f"block{i}", f"block{i-1}"))
    goal = And(*[("On", "block0", "block1")])

    stream_map = {
        "pickfromtable-constraint-sampler": _create_stream_fn(
            rng=rng,
            sample_action_fn=_pickfromtable_sampler,
            ml_next_state_fn=_pickfromtable_ns,
            num_objs_in_constraint=1,
            pre_attrs=["?block-posex", "?block-posey", "?block-held", "?block-clear"],
            eff_attrs=["?block-posex", "?block-posey", "?block-held"]),
        "placeontable-constraint-sampler": _create_stream_fn(
            rng=rng,
            sample_action_fn=_placeontable_sampler,
            ml_next_state_fn=_placeontable_ns,
            num_objs_in_constraint=1,
            pre_attrs=["?block-posex", "?block-posey", "?block-held", "?block-clear"],
            eff_attrs=["?block-posex", "?block-posey", "?block-held"]),
        "unstack-constraint-sampler": _create_stream_fn(
            rng=rng,
            sample_action_fn=_unstack_sampler,
            ml_next_state_fn=_unstack_ns,
            num_objs_in_constraint=2,
            pre_attrs=["?block1-posex", "?block1-posey", "?block1-held", "?block1-clear",
                       "?block2-posex", "?block2-posey", "?block2-held", "?block2-clear"],
            eff_attrs=["?block1-posex", "?block1-posey", "?block1-held", "?block2-clear"]),
        "stack-constraint-sampler": _create_stream_fn(
            rng=rng,
            sample_action_fn=_stack_sampler,
            ml_next_state_fn=_stack_ns,
            num_objs_in_constraint=2,
            pre_attrs=["?block1-posex", "?block1-posey", "?block1-held", "?block1-clear",
                       "?block2-posex", "?block2-posey", "?block2-held", "?block2-clear"],
            eff_attrs=["?block1-posex", "?block1-posey", "?block1-held", "?block2-clear"]),
    }

    constant_map = {}
    pddl_problem = PDDLProblem(domain_pddl, constant_map, stream_pddl,
                               stream_map, init, goal)

    stream_info = {k: StreamInfo(eager=True, p_success=1, opt_gen_fn=PartialInputs(unique=False))
                   for k in stream_map}
    with Profiler():
        solution = solve_focused(pddl_problem, stream_info=stream_info,
                                 max_skeletons=None, max_time=1000, unit_costs=True)
        #solution = solve_incremental(pddl_problem, max_time=1000, unit_costs=True, verbose=True)
    print_solution(solution)

def main():
    start_time = time.time()
    _run()
    print("Universe time:", time.time() - start_time, "seconds")


if __name__ == "__main__":
    main()
