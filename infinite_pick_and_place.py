#!/usr/bin/env python
from conversion import convert_head, convert_expression, pddl_from_expression, EQ, AND, NOT, \
    evaluations_from_init, get_pddl_problem
from fast_downward import run_fast_downward, parse_lisp, parse_domain, write_pddl
from problem import Stream, Object
from collections import namedtuple

# TODO: each action would be associated with a control primitive anyways

DOMAIN_PDDL = """
(define (domain pick-and-place)
  (:requirements :strips :equality)
  (:predicates 
    (Block ?b)
    (Pose ?p)
    (Conf ?q)
    (AtPose ?p ?q)
    (AtConf ?q)
    (Holding ?b)
    (HandEmpty)
  )
  (:action move
    :parameters (?q1 ?a2)
    :precondition (and (Conf ?q1) (Conf ?q2) (AtConf ?q1))
    :effect (and (AtConf ?q2)
                 (not (AtConf ?q1)))
  )
  (:action pick
    :parameters (?b ?p)
    :precondition (and (AtPose ?b ?p) (HandEmpty))
    :effect (and (Holding ?b)
                 (not (AtPose ?b ?p)) (not (HandEmpty)))
  )
  (:action place
    :parameters (?b ?p)
    :precondition (Holding ?b)
    :effect (and (AtPose ?b ?p) (HandEmpty)
                 (not (Holding ?b)))
  )
)
"""

STREAM_PDDL = """
(define (stream pick-and-place)
  (:stream sample-pose
    :inputs ()
    :domain ()
    :outputs (?p)
    :certified (and (Pose ?p))
  )
  (:stream inverse-kinematics
    :inputs (?p)
    :domain (Pose ?p)
    :outputs (?q)
    :certified (and (Conf ?q) (Kin ?q ?p))
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


def get_problem1(n_blocks=1, n_poses=2):
    assert(n_blocks <= n_poses)
    blocks = ['block{}'.format(i) for i in range(n_blocks)]
    poses = [(x, 0) for x in range(n_poses)]
    conf = (5, 0)

    #objects = []
    init = [
        ('Conf', conf),
        ('AtConf', conf),
        ('HandEmpty',),
        #(NOT, ('Holding', blocks[0])),  # Confirms that not
        (EQ, ('total-cost',), 0),
    ]

    init += [('Block', b) for b in blocks]
    init += [('Pose', p) for p in poses]
    init += [('AtPose', b, p) for b, p in zip(blocks, poses)]

    #goal = (AND,
    #        ('Holding', blocks[0]),
    #        (NOT, ('HandEmpty',)))
    goal = ('AtPose', blocks[0], poses[1])

    domain_pddl = DOMAIN_PDDL
    stream_pddl = STREAM_PDDL
    streams = {
        'sample-pose': (lambda: (((x, 0),) for x in range(n_blocks, n_poses))),
        'inverse-kinematics': (lambda p: [((p[0], p[1]+1),)]),
    }
    constants = {}

    return init, goal, domain_pddl, stream_pddl, streams, constants

def obj_from_pddl_plan(pddl_plan):
    if pddl_plan is None:
        return None
    return [(action, map(Object.from_name, args)) for action, args in pddl_plan]

def value_from_obj_plan(obj_plan):
    if obj_plan is None:
        return None
    return [(action, [a.value for a in args]) for action, args in obj_plan]

STREAM_ATTRIBUTES = [':stream', ':inputs', ':domain', ':outputs', ':certified']
Stream = namedtuple('Stream', ['name', 'inputs', 'gen_fn', 'domain', 'outputs'])

def parse_stream(stream_pddl, streams):
    stream_iter = iter(parse_lisp(stream_pddl))
    assert('define' == next(stream_iter))
    pddl_type, stream_name = next(stream_iter)
    assert('stream' == pddl_type)

    for stream in stream_iter:
        attributes = [stream[i] for i in range(0, len(stream), 2)]
        assert(STREAM_ATTRIBUTES == attributes)
        name, inputs, domain, outputs, certified = [stream[i] for i in range(1, len(stream), 2)]
        if name not in streams:
            raise ValueError('Undefined stream conditional generator: {}'.format(name))
        yield Stream(name, inputs, streams[name], outputs, certified)

def solve_finite(problem, **kwargs):
    init, goal, domain_pddl, stream_pddl, streams, constants = problem
    evaluations = evaluations_from_init(init)
    goal_expression = convert_expression(goal)
    domain = parse_domain(domain_pddl)
    problem_pddl = get_pddl_problem(evaluations, goal_expression,
                                    domain_name=domain.name)
    print(list(parse_stream(stream_pddl, streams)))

    return value_from_obj_plan(obj_from_pddl_plan(
        run_fast_downward(domain_pddl, problem_pddl)))

def main():
    problem = get_problem1()
    print(solve_finite(problem))

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
