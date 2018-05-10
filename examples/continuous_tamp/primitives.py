import math

import numpy as np

from examples.continuous_tamp.viewer import SUCTION_HEIGHT

BLOCK_WIDTH = 2
BLOCK_HEIGHT = BLOCK_WIDTH
GRASP = -np.array([0, BLOCK_HEIGHT + SUCTION_HEIGHT/2]) # TODO: side grasps
SCALE_COST = 1.


def scale_cost(cost):
    return int(math.ceil(SCALE_COST*cost))


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

# def get_pose_generator(regions):
#     class PoseGenerator(Generator):
#         def __init__(self, *inputs):
#             super(PoseGenerator, self).__init__()
#             self.b, self.r = inputs
#         def generate(self, outputs=None, streams=tuple()):
#             # TODO: designate which streams can be handled
#             placed = {}
#             for stream in streams:
#                 name, args = stream[0], stream[1:]
#                 if name in ['collision-free', 'cfree']:
#                     for i in range(0, len(args), 2):
#                         b, p = args[i:i+2]
#                         if self.b != b:
#                             placed[b] = p
#             #p = sample_region(self.b, regions[self.r])
#             p = rejection_sample_region(self.b, regions[self.r], placed=placed)
#             if p is None:
#                 return []
#             return [(p,)]
#     return PoseGenerator


def collision_test(b1, p1, b2, p2):
    return interval_overlap(get_block_interval(b1, p1), get_block_interval(b2, p2))


def distance_fn(q1, q2):
    ord = 1  # 1 | 2
    return scale_cost(np.linalg.norm(q2 - q1, ord=ord))


def inverse_kin_fn(b, p):
    return (p - GRASP,)


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
    t = [q1, q2]
    #t = np.vstack([q1, q2])
    return (t,)