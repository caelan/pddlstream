from __future__ import print_function

import numpy as np

from examples.continuous_tamp.primitives import BLOCK_WIDTH, GRASP, sample_region, plan_motion
from pddlstream.utils import hash_or_id
from pddlstream.language.constants import partition_facts, NOT, MINIMIZE, get_constraints
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

    def fn(outputs, facts, hint={}):
        # TODO: pass in the variables and constraint streams instead?
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
            #diagnose_infeasibility_test(model, get_constraints(facts))
            #infeasible = identify_infeasibility(facts, model)
            #infeasible = elastic_filter(model, facts)
            infeasible = deletion_filter(model, get_constraints(facts))
            return OptimizerOutput(infeasible=[infeasible])
        assignment = tuple(value_from_var(get_var(out)) for out in outputs)
        return OptimizerOutput(assignments=[assignment])
    return fn

##################################################

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

def identify_infeasibility(facts, model):
    # TODO: search over irreducible infeasible sets
    model.setObjective(0.0)
    model.computeIIS()
    assert model.IISMinimal # Otherwise return all facts
    #infeasible = set(range(len(facts)))
    infeasible = {int(c.constrName) for c in model.getConstrs() if c.IISConstr}
    infeasible_facts = [facts[index] for index in sorted(infeasible)]
    print('Inconsistent:', infeasible_facts)
    return infeasible

def constraints_from_indices(model, indices):
    names = {str(i) for i in indices}
    return [c for c in model.getConstrs() if c.constrName in names]

def relax_constraints(model, facts, elastic_indices=None):
    from gurobipy import GRB
    # http://www.sce.carleton.ca/faculty/chinneck/docs/ChinneckDravnieks.pdf
    model.setObjective(0.0)
    #model = model.feasibility()
    if elastic_indices is None:
        elastic_indices = set(range(len(facts)))
    elastic_constraints = constraints_from_indices(model, elastic_indices)
    #objective = model.feasRelaxS(relaxobjtype=1,  # relax linear constraints
    #                             minrelax=True,  # Minimum relaxation
    #                             vrelax=False,  # Variable violations
    #                             crelax=True) # Constraint violations
    objective = model.feasRelax(relaxobjtype=0,
                                minrelax=True,
                                vars=[],
                                lbpen=[],
                                ubpen=[],
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
    violated_indices = {index for index, value in violations.items() if 1e-6 < value}
    violated_facts = [facts[index] for index in violated_indices]
    print('Violated:', violated_facts)
    #assignment = {out: value_from_var(get_var(out)) for out in outputs}
    #print('Relaxed Assignment:', assignment)
    return violated_indices

def elastic_filter(original_model, terms):
    elastic = {i for i, term in enumerate(terms) if term[0] != MINIMIZE}
    infeasible = set()
    while True:
        relaxed_model = copy_model(original_model)
        violated = relax_constraints(relaxed_model, terms, elastic)
        if violated is None:
            break
        elastic -= violated
        infeasible |= violated
    infeasible_facts = [terms[index] for index in infeasible]
    print('Infeasible:', infeasible_facts)
    return infeasible

def copy_model(model):
    model.update()
    return model.copy()

def deletion_filter(original_model, terms, indices=None):
    # TODO: apply to general constraint networks
    from gurobipy import GRB
    if indices is None:
        indices = set(range(len(terms)))
    model = copy_model(original_model)
    model.setObjective(0.0)
    #model = model.feasibility()
    prune_constraints = set(model.getConstrs()) - set(constraints_from_indices(model, indices))
    for c in prune_constraints:
        model.remove(c)
    #model.optimize()
    #print(model.status, GRB.INFEASIBLE)

    inconsistent = set()
    for index in sorted(indices):
        temp_model = copy_model(model)
        constraints = constraints_from_indices(temp_model, {index})
        print(constraints)
        if not constraints:
            continue
        for c in constraints:
            temp_model.remove(c)
        temp_model.optimize()
        print(temp_model.status, GRB.INFEASIBLE)
        if temp_model.status == GRB.INFEASIBLE:
            model = temp_model
        else:
            inconsistent.add(index)
    inconsistent_facts = [terms[index] for index in inconsistent]
    print('Infeasible:', inconsistent_facts)
    raw_input('awef')
    return inconsistent

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