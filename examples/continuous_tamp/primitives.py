from collections import namedtuple

import numpy as np

GROUND_NAME = 'grey'
BLOCK_WIDTH = 2
BLOCK_HEIGHT = BLOCK_WIDTH

SUCTION_HEIGHT = 1.
GRASP = -np.array([0, BLOCK_HEIGHT + SUCTION_HEIGHT/2]) # TODO: side grasps
CARRY_Y = 5.
MOVE_COST = 10.
COST_PER_DIST = 1.


def interval_contains(i1, i2):
    """
    :param i1: The container interval
    :param i2: The possibly contained interval
    :return:
    """
    return (i1[0] <= i2[0]) and (i2[1] <= i1[1])


def interval_overlap(i1, i2):
    return (i2[0] <= i1[1]) and (i1[0] <= i2[1])


def get_block_interval(b, p):
    return p[0]*np.ones(2) + np.array([-BLOCK_WIDTH, +BLOCK_WIDTH]) / 2.

##################################################

def collision_test(b1, p1, b2, p2):
    return interval_overlap(get_block_interval(b1, p1), get_block_interval(b2, p2))


def distance_fn(q1, q2):
    ord = 1  # 1 | 2
    return MOVE_COST + COST_PER_DIST*np.linalg.norm(q2 - q1, ord=ord)


def inverse_kin_fn(b, p):
    return (p - GRASP,)


def unreliable_ik_fn(b, p):
    # For testing the algorithms
    while 1e-2 < np.random.random():
        yield None
    yield inverse_kin_fn(b, p)


def get_region_test(regions):
    def test(b, p, r):
        return interval_contains(regions[r], get_block_interval(b, p))
    return test


def sample_region(b, region):
    x1, x2 = np.array(region, dtype=float) - get_block_interval(b, np.zeros(2))
    if x2 < x1:
        return None
    x = np.random.uniform(x1, x2)
    return np.array([x, 0])


def rejection_sample_region(b, region, placed={}, max_attempts=10):
    for _ in range(max_attempts):
        p = sample_region(b, region)
        if p is None:
            break
        if not any(collision_test(b, p, b2, p2) for b2, p2 in placed.items()):
            return p
    return None


def rejection_sample_placed(block_poses={}, block_regions={}, regions={}, max_attempts=10):
    assert(not set(block_poses.keys()) & set(block_regions.keys()))
    for _ in range(max_attempts):
        placed = block_poses.copy()
        remaining = block_regions.items()
        np.random.shuffle(remaining)
        for b, r in remaining:
            p = rejection_sample_region(b, regions[r], placed)
            if p is None:
                break
            placed[b] = p
        else:
            return placed
    return None


def get_pose_gen(regions):
    def gen_fn(b, r):
        while True:
            p = sample_region(b, regions[r])
            if p is None:
                break
            yield (p,)
    return gen_fn


def plan_motion(q1, q2):
    x1, y1 = q1
    x2, y2 = q2
    t = [q1,
         np.array([x1, CARRY_Y]),
         np.array([x2, CARRY_Y]),
         q2]
    return (t,)

##################################################

TAMPState = namedtuple('TAMPState', ['conf', 'holding', 'block_poses'])
TAMPProblem = namedtuple('TAMPProblem', ['initial', 'regions', 'goal_conf', 'goal_regions'])

BLOCK_PREFIX = 'b'
REGION_NAME = 'red'
INITIAL_CONF = np.array([-7.5, 5])
GOAL_CONF = None
#GOAL_CONF = INITIAL_CONF

def get_tight_problem(n_blocks=2, n_goals=2):
    regions = {
        GROUND_NAME: (-15, 15),
        REGION_NAME: (5, 10)
    }

    blocks = ['{}{}'.format(BLOCK_PREFIX, i) for i in range(n_blocks)]
    #poses = [np.array([(BLOCK_WIDTH + 1)*x, 0]) for x in range(n_blocks)]
    poses = [np.array([-(BLOCK_WIDTH + 1) * x, 0]) for x in range(n_blocks)]
    #poses = [sample_region(b, regions[GROUND]) for b in blocks]

    initial = TAMPState(INITIAL_CONF, None, dict(zip(blocks, poses)))
    goal_regions = {block: REGION_NAME for block in blocks[:n_goals]}

    return TAMPProblem(initial, regions, GOAL_CONF, goal_regions)


def get_blocked_problem(n_blocks=2, deterministic=True):
    regions = {
        GROUND_NAME: (-15, 15),
        REGION_NAME: (5, 10)
    }

    blocks = ['{}{}'.format(BLOCK_PREFIX, i) for i in range(n_blocks)]

    if deterministic:
        poses = [np.zeros(2), np.array([7.5, 0])]
        block_poses = dict(zip(blocks, poses))
    else:
        block_regions = {blocks[0]: GROUND_NAME}
        block_regions.update({b: REGION_NAME for b in blocks[1:]})
        block_poses = rejection_sample_placed(block_regions=block_regions, regions=regions)

    initial = TAMPState(INITIAL_CONF, None, block_poses)
    goal_regions = {blocks[0]: 'red'}

    return TAMPProblem(initial, regions, GOAL_CONF, goal_regions)

PROBLEMS = {
    'tight': get_tight_problem,
    'blocked': get_blocked_problem,
}

##################################################

def draw_state(viewer, state, colors):
    viewer.clear_state()
    viewer.draw_environment()
    viewer.draw_robot(*state.conf)
    for block, pose in state.block_poses.items():
        x, y = pose
        viewer.draw_block(x, y, BLOCK_WIDTH, BLOCK_HEIGHT,
                          name=block, color=colors[block])
    if state.holding is not None:
        x, y = state.conf + GRASP
        viewer.draw_block(x, y, BLOCK_WIDTH, BLOCK_HEIGHT,
                          name=state.holding, color=colors[state.holding])
    viewer.tk.update()

def get_random_seed():
    return np.random.get_state()[1][0]