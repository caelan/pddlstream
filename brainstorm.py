
def brainstorm():
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