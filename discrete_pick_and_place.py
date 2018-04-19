#!/usr/bin/env python

from pddlstream.problem import solve_pddl_problem, PDDLProblem

from pddlstream.fast_downward import run_fast_downward

DOMAIN_PDDL = """
(define (domain pick-and-place)
  (:requirements :strips :equality)
  (:predicates (AtPose ?x1 ?x2)
               (Holding ?x)
               (HandEmpty)
  )
  (:action pick
    :parameters (?b ?p)
    :precondition (and (AtPose ?b ?p) (HandEmpty))
    :effect (and (Holding ?b)
                 (not (AtPose ?b ?p)) (not (HandEmpty)))
  )
)
"""

PROBLEM_PDDL = """
(define (problem pick-and-place)
   (:domain pick-and-place)
   (:objects block0 {p0})
   (:init 
     (AtPose block0 {p0})
     (HandEmpty)
   )
   (:goal (and (Holding block0)))
)
"""

# @ # $ % [] {} <> || \/
# What if we have #p1 and #p11

def constant(name):
    return '{{{}}}'.format(name)
    #return '{{{}}}'.format(name)

# https://docs.python.org/3.4/library/string.html
# mako/templating?

def main():
    plan = run_fast_downward(DOMAIN_PDDL, PROBLEM_PDDL, verbose=True)
    print(plan)
    #print(constant(1))

    # TODO: constant map could exclude these and just be used for formatting
    object_map = {
        'p0': (0, 0),
    }
    #print(PROBLEM_PDDL.format(**object_map))

    pddl_problem = PDDLProblem(DOMAIN_PDDL, PROBLEM_PDDL, object_map)
    print(solve_pddl_problem(pddl_problem))


if __name__ == '__main__':
    main()
