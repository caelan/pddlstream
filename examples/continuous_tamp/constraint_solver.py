import numpy as np

from examples.continuous_tamp.primitives import BLOCK_WIDTH, GRASP, sample_region, plan_motion
from pddlstream.utils import hash_or_id

MIN_CLEARANCE = 1e-3 # 0 | 1e-3

##################################################

def cfree_motion_fn(outputs, certified):
    if not outputs:
        return None
    assert(len(outputs) == 1)
    q0, q1 = None, None
    placed = {}
    for fact in certified:
        if fact[0] == 'motion':
            if q0 is not None:
                return None
            q0, _, q1 = fact[1:]
        if fact[0] == 'not':
            _, b, p =  fact[1][1:]
            placed[b] = p
    if q0 is None:
        t = []
        return (t,)
    return plan_motion(q0, q1)


def get_cfree_pose_fn(regions):
    def fn(outputs, certified):
        b, r = None, None
        placed = {}
        for fact in certified:
            print(fact)
            if fact[0] == 'contained':
                b, _, r = fact[1:]
            if fact[0] == 'not':
                _, _, b2, p2 =  fact[1][1:]
                placed[b2] = p2
        p = sample_region(b, regions[r])
        return (p,)
    return fn

##################################################

def has_gurobi():
    try:
        import gurobipy
    except ImportError:
        return False
    return True

def value_from_var(vars):
    import gurobipy
    if isinstance(vars, gurobipy.Var):
        return vars.X
    new_vars = list(map(value_from_var, vars))
    if isinstance(vars, np.ndarray):
        return np.array(new_vars)
    return new_vars

# TODO: partition these

def get_optimize_fn(regions, max_time=5, verbose=False):
    if not has_gurobi():
        raise ImportError('This generator requires Gurobi: http://www.gurobi.com/')
    from gurobipy import Model, GRB, quicksum
    def fn(outputs, facts):
        #print(outputs, facts)
        m = Model(name='TAMP')
        m.setParam(GRB.Param.OutputFlag, verbose)
        m.setParam(GRB.Param.TimeLimit, max_time)

        def unbounded_var():
            return m.addVar(lb=-GRB.INFINITY, ub=GRB.INFINITY)

        def np_var(d=2):
            return np.array([unbounded_var() for _ in range(d)])

        var_from_id = {}
        for fact in facts:
            name, args = fact[0], fact[1:]
            if name == 'conf':
                param, = args
                if param not in var_from_id:
                    var_from_id[hash_or_id(param)] = np_var()
            elif name == 'pose':
                _, param = args
                if param not in var_from_id:
                    var_from_id[hash_or_id(param)] = np_var()
            elif name == 'traj':
                raise NotImplementedError()
                #param, = args
                #if param not in var_from_id:
                #    var_from_id[id(param)] = [np_var(), np_var()]

        def get_var(param):
            return var_from_id.get(hash_or_id(param), param)

        def collision_constraint(fact):
            b1, p1, b2, p2 = map(get_var, fact[1:])
            dist = unbounded_var()
            abs_dist = unbounded_var()
            m.addConstr(dist, GRB.EQUAL, p2[0] - p1[0])
            m.addConstr(BLOCK_WIDTH + MIN_CLEARANCE, GRB.LESS_EQUAL, abs_dist)
            m.addGenConstrAbs(abs_dist, dist)  # abs_

        objective_terms = []
        for fact in facts:
            name, args = fact[0], fact[1:]
            if name == 'kin':
                _, q, p = map(get_var, args)
                for i in range(len(q)):
                    m.addConstr(q[i] + GRASP[i], GRB.EQUAL, p[i])
                m.addConstr(p[1], GRB.EQUAL, 0) # IK vs pick/place semantics
            elif name == 'contained':
                _, p, r = args
                px, py = get_var(p)
                x1, x2 = regions[r]
                m.addConstr(x1, GRB.LESS_EQUAL, px - BLOCK_WIDTH / 2)
                m.addConstr(px + BLOCK_WIDTH / 2, GRB.LESS_EQUAL, x2)
                m.addConstr(py, GRB.EQUAL, 0)
            elif name == 'not':
                fact = args[0]
                name, args = fact[0], fact[1:]
                if name == 'posecollision':
                    collision_constraint(fact)
            elif name == 'cfree':
                collision_constraint(fact)
            elif name == 'motion':
                raise NotImplementedError()
                #q1, t, q2 = map(get_var, args)
                #for i in range(len(q1)):
                #    m.addConstr(t[0][i], GRB.EQUAL, q1[i])
                #for i in range(len(q2)):
                #    m.addConstr(t[1][i], GRB.EQUAL, q2[i])
            elif name == 'minimize':
                fact = args[0]
                name, args = fact[0], fact[1:]
                if name == 'distance':
                    q1, q2 = map(get_var, args)
                    for i in range(len(q1)):
                        delta = q2[i] - q1[i]
                        objective_terms.append(delta * delta)
                        # TODO: cost on endpoints and subtract from total cost

        m.setObjective(quicksum(objective_terms), sense=GRB.MINIMIZE)
        #m.write("file.lp")
        m.optimize()
        if m.status in (GRB.INFEASIBLE, GRB.INF_OR_UNBD): # OPTIMAL | SUBOPTIMAL
            # https://www.gurobi.com/documentation/7.5/refman/optimization_status_codes.html
            return None
        output_values = tuple(value_from_var(get_var(out)) for out in outputs)
        return output_values
    return fn

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