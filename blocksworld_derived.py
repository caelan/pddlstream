#!/usr/bin/env python2.7

from fast_downward import run_fast_downward, translate_task, write_pddl


DOMAIN_PDDL = """
(define (domain blocksworld)
  (:requirements :strips :equality)
  (:predicates (clear ?x)
               (on-table ?x)
               (arm-empty)
               (holding ?x)
               (on ?x ?y))

  (:action pickup
    :parameters (?ob)
    :precondition (and (clear ?ob) (on-table ?ob) (arm-empty))
    :effect (and (holding ?ob) (not (on-table ?ob))
                 (not (arm-empty))))

  (:action putdown
    :parameters  (?ob)
    :precondition (and (holding ?ob))
    :effect (and (arm-empty) (on-table ?ob)
                 (not (holding ?ob))))

  (:action stack
    :parameters  (?ob ?underob)
    :precondition (and  (clear ?underob) (holding ?ob))
    :effect (and (arm-empty) (on ?ob ?underob)
                 (not (holding ?ob))))

  (:action unstack
    :parameters  (?ob ?underob)
    :precondition (and (on ?ob ?underob) (clear ?ob) (arm-empty))
    :effect (and (holding ?ob)
                 (not (on ?ob ?underob)) (not (arm-empty))))

  (:derived (clear ?underob) 
    (not (exists (?ob) (on ?ob ?underob))))
)
"""

PROBLEM_PDDL = """
(define (problem pb2)
   (:domain blocksworld)
   (:objects a b)
   (:init 
     (on-table a)
     (on-table b)
     (arm-empty))
   (:goal (and (on a b))))
"""

##################################################

# TODO: read PDDL from a file or string
# Can always parse name, parameters, etc from pddl
# No types to start but later can extend
# Not assuming any special preconditions and effects makes it easy to extend to other PDDL variants
# Can extend problem file as well if provided with an object map

def main():
    domain_path, problem_path = write_pddl(DOMAIN_PDDL, PROBLEM_PDDL)
    task = translate_task(domain_path, problem_path) # TODO: might need to make these wrt temp
    print(task.objects)
    print(task.axioms) # Separated but not negated
    task.dump()
    #print(task.__dict__)
    #return
    import sys
    # TODO: could even directly convert and mutate the task

    plan = run_fast_downward(DOMAIN_PDDL, PROBLEM_PDDL, verbose=True)
    print(plan)
    print(sys.argv)


if __name__ == '__main__':
    main()
