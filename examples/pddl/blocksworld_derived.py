#!/usr/bin/env python2.7

from __future__ import print_function

from pddlstream.fast_downward import solve_from_pddl, translate_paths, write_pddl

DOMAIN_PDDL = """
(define (domain blocksworld)
  (:requirements :strips :equality)
  (:predicates (on-table ?x)
               (arm-empty)
               (holding ?x)
               (on ?x ?y)
               (unsafe ?y)
  )
  (:action pickup
    :parameters (?ob)
    :precondition (and (on-table ?ob) (arm-empty) (not (unsafe ?ob)))
    :effect (and (holding ?ob) (not (on-table ?ob))
                 (not (arm-empty))))
  (:action putdown
    :parameters  (?ob)
    :precondition (and (holding ?ob))
    :effect (and (arm-empty) (on-table ?ob)
                 (not (holding ?ob))))
  (:action stack
    :parameters  (?ob ?underob)
    :precondition (and (holding ?ob) (not (unsafe ?underob)))
    :effect (and (arm-empty) (on ?ob ?underob)
                 (not (holding ?ob))))
  (:action unstack
    :parameters  (?ob ?underob)
    :precondition (and (on ?ob ?underob) (arm-empty) (not (unsafe ?ob)))
    :effect (and (holding ?ob)
                 (not (on ?ob ?underob)) (not (arm-empty))))
  (:derived (unsafe ?underob) 
    (exists (?ob) (on ?ob ?underob)))
)
"""

PROBLEM_PDDL = """
(define (problem pb2)
   (:domain blocksworld)
   (:objects a b)
   (:init 
     (on b a)
     (on-table a)
     (arm-empty))
   (:goal (and (on a b))))
"""

##################################################

# TODO: read PDDL from a file or string
# Can always parse name, parameters, etc from pddl
# No types to start but later can extend
# Not assuming any special preconditions and effects makes it easy to extend to other PDDL variants
# Can extend problem file as well if provided with an object map

def brainstorm():
    domain_path, problem_path = write_pddl(DOMAIN_PDDL, PROBLEM_PDDL)
    task = translate_paths(domain_path, problem_path) # TODO: might need to make these wrt temp
    print(task.objects)
    print(task.axioms) # Separated but not negated
    task.dump()
    #print(task.__dict__)
    #return
    import sys
    # TODO: could even directly convert and mutate the task

def main():
    plan, cost = solve_from_pddl(DOMAIN_PDDL, PROBLEM_PDDL, debug=False)
    print('Plan:', plan)
    print('Cost:', cost)

if __name__ == '__main__':
    main()
