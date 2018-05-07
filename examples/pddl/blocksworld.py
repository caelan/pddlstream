#!/usr/bin/env python2.7

from __future__ import print_function

from pddlstream.fast_downward import solve_from_pddl, translate_paths, write_pddl, parse_domain
from pddlstream.utils import read
import os

##################################################

# TODO: read PDDL from a file or string
# Can always parse name, parameters, etc from pddl
# No types to start but later can extend
# Not assuming any special preconditions and effects makes it easy to extend to other PDDL variants
# Can extend problem file as well if provided with an object map
# TODO: could extract the FD parser by itself
# TODO: include my version of FD as a submodule

def main():
    directory = os.path.dirname(os.path.abspath(__file__))
    domain_pddl = read(os.path.join(directory, 'domain.pddl'))
    problem_pddl = read(os.path.join(directory, 'problem.pddl'))

    plan, cost = solve_from_pddl(domain_pddl, problem_pddl, debug=False)
    print('Plan:', plan)
    print('Cost:', cost)

if __name__ == '__main__':
    main()
