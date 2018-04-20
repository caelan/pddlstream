#!/usr/bin/env python2.7

from __future__ import print_function

from pddlstream.fast_downward import solve_from_pddl, translate_pddl, write_pddl, parse_domain

DOMAIN_PDDL = '''
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
    :effect (and (holding ?ob) (not (clear ?ob)) (not (on-table ?ob))
                 (not (arm-empty))))

  (:action putdown
    :parameters  (?ob)
    :precondition (and (holding ?ob))
    :effect (and (clear ?ob) (arm-empty) (on-table ?ob)
                 (not (holding ?ob))))

  (:action stack
    :parameters  (?ob ?underob)
    :precondition (and  (clear ?underob) (holding ?ob))
    :effect (and (arm-empty) (clear ?ob) (on ?ob ?underob)
                 (not (clear ?underob)) (not (holding ?ob))))

  (:action unstack
    :parameters  (?ob ?underob)
    :precondition (and (on ?ob ?underob) (clear ?ob) (arm-empty))
    :effect (and (holding ?ob) (clear ?underob)
                 (not (on ?ob ?underob)) (not (clear ?ob)) (not (arm-empty)))))
'''

PROBLEM_PDDL = '''
(define (problem pb2)
   (:domain blocksworld)
   (:objects a b)
   (:init 
     (on-table a)
     (on-table b)
     (clear a)
     (clear b)
     (arm-empty))
   (:goal (and (on a b))))
'''


##################################################

# TODO: read PDDL from a file or string
# Can always parse name, parameters, etc from pddl
# No types to start but later can extend
# Not assuming any special preconditions and effects makes it easy to extend to other PDDL variants
# Can extend problem file as well if provided with an object map
# TODO: could extract the FD parser by itself
# TODO: include my version of FD as a submodule


def brainstorm():
    #print(DOMAIN_PDDL)
    #print(PROBLEM_PDDL)
    #print(parse_lisp(DOMAIN_PDDL.encode('latin-1')))
    #print(parse_lisp(DOMAIN_PDDL.encode('ISO-8859-1')))
    #print(parse_lisp(PROBLEM_PDDL))
    #print(parse_domain(DOMAIN_PDDL))
    #print(parse_domain(parse_lisp(DOMAIN_PDDL)))
    #return

    domain_path, problem_path = write_pddl(DOMAIN_PDDL, PROBLEM_PDDL)
    print(parse_domain(domain_path))

    task = translate_pddl(domain_path, problem_path) # TODO: might need to make these wrt temp
    print(task.objects)
    task.dump()
    #print(task.__dict__)
    #return
    import sys
    # TODO: could even directly convert and mutate the task
    return

def main():
    plan, cost = solve_from_pddl(DOMAIN_PDDL, PROBLEM_PDDL, debug=False)
    print('Plan:', plan)
    print('Cost:', cost)

if __name__ == '__main__':
    main()
