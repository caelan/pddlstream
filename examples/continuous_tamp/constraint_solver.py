from __future__ import print_function

import numpy as np

from examples.continuous_tamp.primitives import BLOCK_WIDTH, GRASP, sample_region, plan_motion
from pddlstream.utils import hash_or_id
from pddlstream.language.constants import partition_facts, NOT, MINIMIZE
from pddlstream.language.optimizer import OptimizerOutput

MIN_CLEARANCE = 1e-3 # 0 | 1e-3

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
    # https://www.gurobi.com/documentation/8.1/examples/diagnose_and_cope_with_inf.html
    # https://www.gurobi.com/documentation/8.1/examples/tsp_py.html#subsubsection:tsp.py
    # https://examples.xpress.fico.com/example.pl?id=Infeasible_python
    # https://www.fico.com/fico-xpress-optimization/docs/latest/examples/python/GUID-E77AC35C-9488-3F0A-98B4-7F5FD81AFF1D.html
    if not has_gurobi():
        raise ImportError('This generator requires Gurobi: http://www.gurobi.com/')
    from gurobipy import Model, GRB, quicksum

    def fn(outputs, facts, fluents={}, hint={}):
        # TODO: pass in the variables and constraint streams instead?
        # TODO: fluents is map from constraint to fluent inputs
        # The true test is placing two blocks in a tight region obstructed by one

        print('Constraints:', facts)
        model = Model(name='TAMP')
        model.setParam(GRB.Param.OutputFlag, verbose)
        model.setParam(GRB.Param.TimeLimit, max_time)

        def unbounded_var():
            return model.addVar(lb=-GRB.INFINITY, ub=GRB.INFINITY)

        def np_var(d=2):
            return np.array([unbounded_var() for _ in range(d)])

        var_from_id = {}
        for fact in facts:
            prefix, args = fact[0], fact[1:]
            if prefix == 'conf':
                param, = args
                if param not in var_from_id:
                    var_from_id[hash_or_id(param)] = np_var()
            elif prefix == 'pose':
                _, param = args
                if param not in var_from_id:
                    var_from_id[hash_or_id(param)] = np_var()
            elif prefix == 'traj':
                raise NotImplementedError()
                #param, = args
                #if param not in var_from_id:
                #    var_from_id[id(param)] = [np_var(), np_var()]

        def get_var(param):
            return var_from_id.get(hash_or_id(param), param)

        def collision_constraint(args, name):
            b1, p1, b2, p2 = map(get_var, args)
            dist = unbounded_var()
            abs_dist = unbounded_var()
            model.addConstr(dist, GRB.EQUAL, p2[0] - p1[0], name=name)
            model.addConstr(BLOCK_WIDTH + MIN_CLEARANCE, GRB.LESS_EQUAL, abs_dist, name=name)
            model.addGenConstrAbs(abs_dist, dist, name=name)  # abs_

        #postive, negative, functions = partition_facts(facts)
        objective_terms = []
        for index, fact in enumerate(facts):
            prefix, args = fact[0], fact[1:]
            name = str(index)
            if prefix == 'kin':
                _, q, p = map(get_var, args)
                for i in range(len(q)):
                    model.addConstr(q[i] + GRASP[i], GRB.EQUAL, p[i], name=name)
                model.addConstr(p[1], GRB.EQUAL, 0, name=name) # IK vs pick/place semantics
            elif prefix == 'contained':
                _, p, r = args
                px, py = get_var(p)
                x1, x2 = regions[r]
                model.addConstr(x1, GRB.LESS_EQUAL, px - BLOCK_WIDTH / 2, name=name)
                model.addConstr(px + BLOCK_WIDTH / 2, GRB.LESS_EQUAL, x2, name=name)
                model.addConstr(py, GRB.EQUAL, 0, name=name)
            elif prefix == NOT:
                fact = args[0]
                predicate, args = fact[0], fact[1:]
                if predicate == 'posecollision':
                    collision_constraint(args, name)
            elif prefix == 'cfree':
                collision_constraint(args, name)
            elif prefix == 'motion':
                raise NotImplementedError()
                #q1, t, q2 = map(get_var, args)
                #for i in range(len(q1)):
                #    m.addConstr(t[0][i], GRB.EQUAL, q1[i], name=name)
                #for i in range(len(q2)):
                #    m.addConstr(t[1][i], GRB.EQUAL, q2[i], name=name)
            elif prefix == MINIMIZE:
                fact = args[0]
                func, args = fact[0], fact[1:]
                if func == 'distance':
                    q1, q2 = map(get_var, args)
                    for i in range(len(q1)):
                        delta = q2[i] - q1[i]
                        objective_terms.append(delta * delta)
                        # TODO: cost on endpoints and subtract from total cost

        for out, value in hint.items():
            for var, coord in zip(get_var(out), value):
                var.start = coord

        model.setObjective(quicksum(objective_terms), sense=GRB.MINIMIZE)
        #m.write("file.lp")
        model.optimize()
        # https://www.gurobi.com/documentation/7.5/refman/optimization_status_codes.html
        if model.status in (GRB.INFEASIBLE, GRB.INF_OR_UNBD): # OPTIMAL | SUBOPTIMAL
            #diagnose_infeasibility(outputs, facts, m, get_var)
            infeasible = identify_infeasibility(facts, model)
            return OptimizerOutput(infeasible=[infeasible])
        assignment = tuple(value_from_var(get_var(out)) for out in outputs)
        return OptimizerOutput(assignments=[assignment])
    return fn

def identify_feasible_subsets(facts, model):
    # TODO: add facts corresponding to feasible subproblems
    raise NotImplementedError()

def identify_infeasibility(facts, model):
    # TODO: search over irreducible infeasible sets
    model.computeIIS()
    assert model.IISMinimal # Otherwise return all facts
    #infeasible = set(range(len(facts)))
    infeasible = {int(c.constrName) for c in model.getConstrs() if c.IISConstr}
    infeasible_facts = [facts[index] for index in sorted(infeasible)]
    print('Infeasible:', infeasible_facts)
    return infeasible

def diagnose_infeasibility_test(outputs, facts, model, get_var):
    # TODO: drop the objective function and decompose into smaller clusters
    # TODO: is there a way of determining which constraints are weak (separating plane on set of values?)
    # https://ac.els-cdn.com/0377221781901776/1-s2.0-0377221781901776-main.pdf?_tid=c2247453-b8b8-4f5d-b4e5-77ee8fa2d109&acdnat=1548267058_2548c4fa1e9dfba6e7ce0cf88d35f6e1
    # http://www.sce.carleton.ca/faculty/chinneck/docs/ChinneckDravnieks.pdf

    model.setObjective(0.0)
    model.computeIIS()
    if model.IISMinimal:
        print('IIS is minimal\n')
    else:
        print('IIS is not minimal\n')
    iss_constraints = {c.constrName for c in model.getConstrs() if c.IISConstr}
    iss_facts = [facts[int(name)] for name in sorted(iss_constraints)]
    print(iss_facts)
    for c in model.getConstrs():
        if c.constrName in iss_constraints:
            model.remove(c)
    # TODO: reoptimize

    # Iterate over subsets of constraints that are allowed to be violated/removed
    # The relaxation is a heuristic to more intelligently guide this search
    # Use L1 penalization to enforce sparsity. Could even frame as a MILP
    # The assumption is that there is intersection between the failure and a solution

    #m.feasRelax
    model.feasRelaxS(relaxobjtype=0,  # relax linear constraints
                 minrelax=False,  # Minimum relaxation
                 vrelax=False,  # Variable violations
                 crelax=True) # Constraint violations
    model.optimize()
    art_vars = [v for v in model.getVars() if (0 < v.x) and
                (v.varname.startswith('ArtP_') or v.varname.startswith('ArtN_'))]
    violated_constraints = {v.varname[5:] for v in art_vars}
    violated_facts = [facts[int(name)] for name in sorted(violated_constraints)]
    print(violated_facts)
    print(tuple(value_from_var(get_var(out)) for out in outputs))
    raw_input('Failure!')

##################################################

def cfree_motion_fn(outputs, facts, hint={}):
    if not outputs:
        return None
    assert(len(outputs) == 1)
    # TODO: handle connected components
    q0, q1 = None, None
    placed = {}
    for fact in facts:
        if fact[0] == 'motion':
            if q0 is not None:
                return None
            q0, _, q1 = fact[1:]
        if fact[0] == NOT:
            _, b, p =  fact[1][1:]
            placed[b] = p
    if q0 is None:
        t = []
        return (t,)
    return plan_motion(q0, q1)

##################################################

def get_cfree_pose_fn(regions):
    def fn(outputs, certified):
        b, r = None, None
        placed = {}
        for fact in certified:
            if fact[0] == 'contained':
                b, _, r = fact[1:]
            if fact[0] == NOT:
                _, _, b2, p2 = fact[1][1:]
                placed[b2] = p2
        p = sample_region(b, regions[r])
        return (p,)

    return fn

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