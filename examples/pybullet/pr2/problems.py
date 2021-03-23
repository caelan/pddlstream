from __future__ import print_function

from examples.pybullet.utils.pybullet_tools.pr2_problems import create_pr2, create_table, Problem
from examples.pybullet.utils.pybullet_tools.pr2_utils import get_other_arm, get_carry_conf, set_arm_conf, open_arm, \
    arm_conf, REST_LEFT_ARM, close_arm, set_group_conf
from examples.pybullet.utils.pybullet_tools.utils import get_bodies, sample_placement, pairwise_collision, \
    add_data_path, load_pybullet, set_point, Point, create_box, stable_z, joint_from_name, get_point, wait_for_user,\
    RED, GREEN, BLUE, BLACK, WHITE, BROWN, TAN, GREY

import numpy as np


def sample_placements(body_surfaces, obstacles=None, min_distances={}):
    if obstacles is None:
        obstacles = [body for body in get_bodies() if body not in body_surfaces]
    obstacles = list(obstacles)
    # TODO: max attempts here
    for body, surface in body_surfaces.items():
        min_distance = min_distances.get(body, 0.01)
        while True:
            pose = sample_placement(body, surface)
            if pose is None:
                return False
            if not any(pairwise_collision(body, obst, max_distance=min_distance)
                       for obst in obstacles if obst not in [body, surface]):
                obstacles.append(body)
                break
    return True