from __future__ import print_function

import numpy as np

from examples.continuous_tamp.primitives import BLOCK_WIDTH, sample_region, plan_motion
from pddlstream.language.constants import partition_facts, NOT, MINIMIZE, get_constraints, is_parameter
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

def unbounded_var(model):
    from gurobipy import GRB
    return model.addVar(lb=-GRB.INFINITY, ub=GRB.INFINITY)

def np_var(model, d=2):
    return np.array([unbounded_var(model) for _ in range(d)])

def copy_model(model):
    model.update()
    return model.copy()

##################################################

def collision_constraint(model, name, b1, p1, b2, p2):
    from gurobipy import GRB
    dist = unbounded_var(model)
    abs_dist = unbounded_var(model)
    model.addConstr(dist, GRB.EQUAL, p2[0] - p1[0], name=name)
    model.addConstr(BLOCK_WIDTH + MIN_CLEARANCE, GRB.LESS_EQUAL, abs_dist, name=name)
    model.addGenConstrAbs(abs_dist, dist, name=name)  # abs_

def kinematics_constraint(model, name, b, q, p, g):
    from gurobipy import GRB
    for i in range(len(q)):
        model.addConstr(q[i] + g[i], GRB.EQUAL, p[i], name=name)
    model.addConstr(p[1], GRB.EQUAL, 0, name=name)  # IK vs pick/place semantics

def contained_constraint(model, regions, name, b, p, r):
    from gurobipy import GRB
    px, py = p
    x1, x2 = regions[r]
    model.addConstr(x1, GRB.LESS_EQUAL, px - BLOCK_WIDTH / 2, name=name)
    model.addConstr(px + BLOCK_WIDTH / 2, GRB.LESS_EQUAL, x2, name=name)
    model.addConstr(py, GRB.EQUAL, 0, name=name)

def motion_constraint(model, name, q1, t, q2):
    from gurobipy import GRB
    for i in range(len(q1)):
       model.addConstr(t[0][i], GRB.EQUAL, q1[i], name=name)
    for i in range(len(q2)):
       model.addConstr(t[1][i], GRB.EQUAL, q2[i], name=name)

def distance_cost(q1, q2):
    # TODO: cost on endpoints and subtract from total cost
    terms = []
    for i in range(len(q1)):
        delta = q2[i] - q1[i]
        terms.append(delta * delta)
    return terms

##################################################

def get_optimize_fn(regions, collisions=True, max_time=5, diagnose=True, verbose=False):
    # https://www.gurobi.com/documentation/8.1/examples/diagnose_and_cope_with_inf.html
    # https://www.gurobi.com/documentation/8.1/examples/tsp_py.html#subsubsection:tsp.py
    # https://examples.xpress.fico.com/example.pl?id=Infeasible_python
    # https://www.fico.com/fico-xpress-optimization/docs/latest/examples/python/GUID-E77AC35C-9488-3F0A-98B4-7F5FD81AFF1D.html
    if not has_gurobi():
        raise ImportError('This generator requires Gurobi: http://www.gurobi.com/')
    from gurobipy import Model, GRB, quicksum

    def fn(outputs, facts, hint={}):
        # TODO: pass in the variables and constraint streams instead?
        # The true test is placing two blocks in a tight region obstructed by one
        positive, negative, costs = partition_facts(facts)
        #print('Parameters:', outputs)
        print('Constraints:', positive + negative)
        if costs:
            print('Costs:', costs)
        model = Model(name='TAMP')
        model.setParam(GRB.Param.OutputFlag, verbose)
        model.setParam(GRB.Param.TimeLimit, max_time)

        var_from_param = {}
        for fact in facts:
            prefix, args = fact[0], fact[1:]
            if prefix == 'conf':
                param, = args
                if is_parameter(param):
                    var_from_param[param] = np_var(model)
            elif prefix == 'pose':
                _, param = args
                if is_parameter(param):
                    var_from_param[param] = np_var(model)
            elif prefix == 'traj':
                raise NotImplementedError()
                #param, = args
                #if param not in var_from_id:
                #    var_from_id[id(param)] = [np_var(model), np_var(model)]

        def get_var(p):
            return var_from_param[p] if is_parameter(p) else p

        objective_terms = []
        constraint_from_name = {}
        for index, fact in enumerate(facts):
            prefix, args = fact[0], fact[1:]
            name = str(index)
            if prefix == 'kin':
                kinematics_constraint(model, name, *map(get_var, args))
            elif prefix in ('contain', 'contained'):
                contained_constraint(model, regions, name, *map(get_var, args))
            elif prefix == 'cfree' and collisions:
                collision_constraint(model, name, *map(get_var, args))
            elif prefix == 'motion':
                #motion_constraint(model, name, *map(get_var, args))
                raise NotImplementedError()
            elif prefix == NOT:
                fact = args[0]
                predicate, args = fact[0], fact[1:]
                if predicate == 'posecollision' and collisions:
                    collision_constraint(model, name, *map(get_var, args))
            elif prefix == MINIMIZE:
                fact = args[0]
                func, args = fact[0], fact[1:]
                if func in ('dist', 'distance'):
                    objective_terms.extend(distance_cost(*map(get_var, args)))
                continue
            constraint_from_name[name] = fact

        for out, value in hint.items():
            for var, coord in zip(get_var(out), value):
                var.start = coord

        model.setObjective(quicksum(objective_terms), sense=GRB.MINIMIZE)
        #m.write("file.lp")
        model.optimize()
        # https://www.gurobi.com/documentation/7.5/refman/optimization_status_codes.html
        if model.status in (GRB.INFEASIBLE, GRB.INF_OR_UNBD): # OPTIMAL | SUBOPTIMAL
            if not diagnose:
                return OptimizerOutput()
            constraint_indices = {i for i, term in enumerate(facts) if term[0] != MINIMIZE}
            #infeasible = constraint_indices
            infeasible = compute_inconsistent(model)
            #infeasible = deletion_filter(model, constraint_indices)
            #infeasible = elastic_filter(model, constraint_indices)
            infeasible_facts = [facts[index] for index in sorted(infeasible)]
            print('Inconsistent:', infeasible_facts)
            return OptimizerOutput(infeasible=[infeasible])
        assignment = tuple(value_from_var(get_var(out)) for out in outputs)
        return OptimizerOutput(assignments=[assignment])
    return fn
    #return lambda outputs, facts, **kwargs: \
    #    identify_infeasibility(fn, outputs, facts, diagnose=False, **kwargs)

##################################################

def compute_inconsistent(model):
    # TODO: search over irreducible infeasible sets
    model.setObjective(0.0)
    model.computeIIS()
    #assert model.IISMinimal
    infeasible = {int(c.constrName) for c in model.getConstrs() if c.IISConstr}
    return infeasible

def constraints_from_indices(model, indices):
    names = {str(i) for i in indices}
    return [c for c in model.getConstrs() if c.constrName in names]

def deletion_filter(original_model, indices):
    from gurobipy import GRB
    model = original_model.feasibility()
    prune_constraints = set(model.getConstrs()) - set(constraints_from_indices(model, indices))
    for c in prune_constraints:
        model.remove(c)
    model.optimize()
    assert model.status == GRB.INFEASIBLE

    inconsistent = set()
    for index in sorted(indices):
        temp_model = copy_model(model)
        constraints = constraints_from_indices(temp_model, {index})
        if not constraints:
            continue
        for c in constraints:
            temp_model.remove(c)
        temp_model.optimize()
        if temp_model.status == GRB.INFEASIBLE:
            model = temp_model
        else:
            inconsistent.add(index)
    return inconsistent

def identify_infeasibility(fn, parameters, terms, **kwargs):
    # TODO: apply to general constraint networks
    output = fn(parameters, terms, **kwargs)
    if output:
        return output
    active_indices = {i for i, term in enumerate(terms) if term[0] != MINIMIZE}
    for index in list(active_indices):
        constraints = [terms[i] for i in active_indices - {index}]
        output = fn(parameters, constraints, **kwargs)
        if output:
            active_indices.remove(index)
    # TODO: be careful about removing variables
    infeasible_facts = [terms[index] for index in sorted(active_indices)]
    print('Inconsistent:', infeasible_facts)
    return OptimizerOutput(infeasible=[infeasible_facts])

##################################################

def relax_constraints(model, indices):
    from gurobipy import GRB
    # http://www.sce.carleton.ca/faculty/chinneck/docs/ChinneckDravnieks.pdf
    model.setObjective(0.0)
    elastic_constraints = constraints_from_indices(model, indices)
    objective = model.feasRelax(relaxobjtype=0, # feasRelaxS
                                minrelax=True,
                                vars=[], lbpen=[], ubpen=[],
                                constrs=elastic_constraints,
                                rhspen=[1.]*len(elastic_constraints))
    model.optimize()
    if model.status == GRB.INFEASIBLE:
        return None
    #print('Violation:', objective)
    art_vars = [var for var in model.getVars()
                if any(var.varname.startswith(p) for p in ['ArtP_', 'ArtN_'])] # Positive & Negative
    violations = {}
    for var in art_vars:
        index = int(var.varname[5:])
        violations[index] = violations.get(index, 0) + var.x
    #for index, value in sorted(violations.items(), key=lambda (i, v): v):
    #    print('{}: {}'.format(facts[index], value))
    violated = {index for index, value in violations.items() if 1e-6 < value}
    return violated

def elastic_filter(original_model, indices):
    elastic = set(indices)
    inconsistent = set()
    while True:
        relaxed_model = copy_model(original_model)
        violated = relax_constraints(relaxed_model, elastic)
        if violated is None:
            break
        elastic -= violated
        inconsistent |= violated
    #return inconsistent
    return deletion_filter(original_model, inconsistent)

def identify_feasible_subsets(facts, model):
    # TODO: add facts corresponding to feasible subproblems
    # The trouble is that it's not clear which constraints would be useful to relax
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
    # TODO: drop the objective function and decompose into smaller clusters
    # Iterate over subsets of constraints that are allowed to be violated/removed
    # The relaxation is a heuristic to more intelligently guide this search
    # Use L1 penalization to enforce sparsity. Could even frame as a MILP
    # The assumption is that there is intersection between the failure and a solution
    raise NotImplementedError()

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