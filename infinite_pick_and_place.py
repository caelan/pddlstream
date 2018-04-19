#!/usr/bin/env python
from conversion import convert_head, convert_expression, pddl_from_expression, EQ, AND, NOT, \
    evaluations_from_init, get_pddl_problem
from fast_downward import run_fast_downward, parse_lisp
from problem import Stream, Object

# TODO: each action would be associated with a control primitive anyways

DOMAIN_PDDL = """
(define (domain pick-and-place)
  (:requirements :strips :equality)
  (:predicates 
    (Block ?x1)
    (Pose ?x1)
    (AtPose ?x1 ?x2)
    (Holding ?x1)
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

STREAM_PDDL = """
(define (stream pick-and-place)
  (:stream inverse-kinematics
    :inputs (?b ?p)
    :domain (Pose ?p)
    :outputs (?q)
    :certified (Kin ?q ?p)
  )
)
"""

Stream(inp='?p', domain='(Pose ?p)',
       fn=lambda p: (p,),
       out='?q', certifed='(Kin ?q ?p)'),


# Basic functions for parsing PDDL (Lisp) files.

# @ # $ % [] {} <> || \/
# What if we have #p1 and #p11

# https://docs.python.org/3.4/library/string.html
# mako/templating?

#class Not(object):

# TODO: start by requiring that all objects have a substituion


#def pddl_from_head(head):
#    return pddl_from_expression((head.name,) + head.args)

#def And(*expressions):
#    return (AND,) + expressions

# TODO: I think we do want everything to be an object at the end of the day


def get_problem1():
    block0 = 'block0'
    p0 = (0, 0)

    #objects = []
    init = [
        ('Block', block0),
        ('Pose', p0),
        ('AtPose', block0, p0),
        ('HandEmpty',),
        #(NOT, ('Holding', block0)), # Confirms that not
        (EQ, ('total-cost',), 0),
    ]
    #goal = [
    #    ('Holding', block0),
    #    ('not', ('Holding', block0)),
    #    (NOT, ('Holding', block0)),
    #]
    goal = (AND,
            ('Holding', block0),
            (NOT, ('HandEmpty',)),
            )

    domain_pddl = DOMAIN_PDDL
    stream_pddl = STREAM_PDDL
    streams = {}
    constants = {}

    return init, goal, domain_pddl, stream_pddl, streams, constants



def main():
    problem = get_problem1()
    #print(problem)

    init, goal, domain_pddl, stream_pddl, streams, constants = problem

    print(parse_lisp(domain_pddl))
    print(parse_lisp(stream_pddl))

    goal_expression = convert_expression(goal)

    print(goal_expression)
    print(repr(goal_expression))
    print(str(goal_expression))
    print(pddl_from_expression(goal_expression))

    evaluations = evaluations_from_init(init)
    print(evaluations)

    for evaluation in evaluations:
        print(evaluation)

    problem_pddl = get_pddl_problem(evaluations, goal_expression)

    print(domain_pddl)
    print(problem_pddl)

    plan = run_fast_downward(domain_pddl, problem_pddl)


    return


    #Block = 'Block'
    #Block = Predicate('Block')

    block0 = 'block0'
    p0 = (0, 0)

    def ik(p):
        return p,

    gen_fns = { # TODO: return
        'inverse-kinematics': ik, # TODO: include various options as well
    }





    # TODO: the more I can separate representation language from parsing, the better

    #objects = []
    init = [
        ('Block', block0),
        ('Pose', p0),
        ('AtPose', block0, p0),
        ('HandEmpty',),
    ]
    #goal = [
    #    ('Holding', block0),
    #    ('not', ('Holding', block0)),
    #    (NOT, ('Holding', block0)),
    #]
    goal = (AND,
            ('Holding', block0),
            ('HandEmpty',),
            )

    # TODO: no string constants that aren't associated with a value


    for atom in init:
        args = atom[1:]
        for arg in args:
            obj = Object.from_value(arg)
            print(obj)
        print(convert_head(atom))



    # TODO: can make rule streams that automatically infer object (types in general)
    # TODO: apply the constant mapping here as well
    streams = [
        Stream(inp='?p', domain='(Pose ?p)',
               fn=lambda p: (p,),
               out='?q', certifed='(Kin ?q ?p)'),

        RuleStream(inp='?q ?p', domain='(Kin ?q ?p)', # TODO: infer these from types
               certifed='(and (Pose ?p) (Conf ?q))'),

        Stream(inp='?q1 ?q2', domain='(and (Conf ?q1) (Conf ?q2))',
               fn=lambda q1, q2: (abs(q2 - q1),),
               out='?d', certifed='(= (Distance ?q1 ?q2) ?d)'),
    ]
    # TODO: the difference between predicates and functions is that functions must be evaluated
    # While predicates are always assumed to be false

    # Initial state conditional effects
    streams = [
        Stream(inp='?b', domain='(and ())',
               fn=lambda b: ([((i, 0),)] for i in range(5)),
               out='?p', certifed='( )'),
    ]
    # TODO: maybe I take an even more basic view where certified are just the statements to write down completing the values
    # TODO: then I could have functions and streams do equivalent things

    # TODO: name whatever the output of the stream is but don't require that it be an object.


    # TODO: can make a wrapper around the stream fn to simplify what its behavior is

    # (Predicate ?inp1 ?inp2 ?out1 ?out2)
    # (= (Function) ?out) # Number, True, False

    # TODO: parse streams like a pddl thing
    # TODO: can map stream name to function and any parameters

if __name__ == '__main__':
    main()
