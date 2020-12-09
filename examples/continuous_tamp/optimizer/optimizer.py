from __future__ import print_function

import numpy as np
import random
import time

from examples.continuous_tamp.primitives import BLOCK_WIDTH, sample_region, plan_motion, GRASP
from pddlstream.language.constants import partition_facts, NOT, MINIMIZE, get_constraints, is_parameter
from pddlstream.language.optimizer import OptimizerOutput
from pddlstream.utils import INF, elapsed_time

MIN_CLEARANCE = 1e-3 # 0 | 1e-3

NAN = float('nan')

def has_gurobi():
    try:
        import gurobipy
    except ImportError:
        return False
    return True

def value_from_var(vars):
    import gurobipy
    if isinstance(vars, float):
        return vars
    if isinstance(vars, gurobipy.Var):
        return vars.X # .Xn
    new_vars = list(map(value_from_var, vars))
    if isinstance(vars, np.ndarray):
        return np.array(new_vars)
    return new_vars

def unbounded_var(model):
    from gurobipy import GRB
    return model.addVar(lb=-GRB.INFINITY, ub=GRB.INFINITY)

def np_var(model, d=2, lower=None, upper=None):
    from gurobipy import GRB
    if lower is None:
        lower = d*[-GRB.INFINITY]
    if upper is None:
        upper = d*[+GRB.INFINITY]
    return np.array([model.addVar(lb=lb, ub=ub) for lb, ub in zip(lower, upper)])

def copy_model(model):
    model.update()
    return model.copy()

def vars_from_expr(expr):
    return [expr.getVar(i) for i in range(expr.size())]

def set_value(var, value):
    var.LB = value
    var.UB = value

def set_guess(var, value, hard=True):
    if hard:
        # solver will try to build a single feasible solution from the provided set of variable values
        var.Start = value
    else:
        # variable hints provide guidance to the MIP solver that affects the entire solution process
        var.VarHintVal = value

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
    #model.addConstr(p[1], GRB.EQUAL, 0, name=name)  # IK vs pick/place semantics

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

def distance_cost(model, q1, q2, norm=1):
    from gurobipy import GRB
    # TODO: cost on endpoints and subtract from total cost
    terms = []
    for i in range(len(q1)):
        delta = q2[i] - q1[i]
        if norm == 1:
            # TODO: doesn't work with deletion filter (additional constraints)
            distance = model.addVar(lb=0., ub=GRB.INFINITY)
            model.addConstr(-delta <= distance)
            model.addConstr(delta <= distance)
            terms.append(distance)
        elif norm == 2:
            terms.append(delta * delta)
        else:
            raise RuntimeError(norm)
    return terms

##################################################

def sample_sphere_surface(d, uniform=True):
    # TODO: hyperspherical coordinates
    # https://en.wikipedia.org/wiki/N-sphere#Spherical_coordinates
    while True:
        v = np.random.randn(d)
        r = np.sqrt(v.dot(v))
        if not uniform or (r <= 1.):
            return v / r

def sample_sphere(d, **kwargs):
    v = sample_sphere_surface(d, **kwargs)
    r = np.random.rand()
    return np.power(r, 1./d)*v

def sample_subspace(d, m):
    # TODO: linear spaces sampling method
    # https://arxiv.org/abs/1810.06271
    A = np.random.randn(m, d)
    b = np.random.randn(m)
    return A, b

##################################################

def sample_targets(model, variables):
    # TODO: project without inequality constraints (placement)
    # TODO: manifold learning for feasible subspace
    # TODO: sample from neighborhood of the previous solution
    from gurobipy import GRB
    model.update()
    objective_terms = []
    for var in variables:
        # TODO: apply only to some variables
        if (-INF < var.LB) and (var.UB < INF):
            # TODO: inequality constraint version of this
            extent = var.UB - var.LB
            value = random.uniform(var.LB, var.UB)
            delta = var - value
            distance = model.addVar(lb=0., ub=GRB.INFINITY)
            model.addConstr(-delta <= distance)
            model.addConstr(delta <= distance)
            objective_terms.append(distance / extent)
            # covar_from_paramord.X = 0 # Attribute 'X' cannot be set
            print(var, var.LB, var.UB, value)
            # TODO: start with feasible solution and then expand
    return objective_terms

def sample_solutions(model, variables, num_samples=INF, norm=2, closest=True):
    from gurobipy import GRB, quicksum, abs_
    start_time = time.time()
    objective = model.getObjective()
    #sprint(objective, objective.getValue())
    # model = model.feasibility()
    # model.setObjective(0.0)
    if norm == 2:
        model.setParam(GRB.Param.NonConvex, 2) # PSDTol
    objective_terms = []
    if closest:
        min_distance = model.addVar(lb=0., ub=GRB.INFINITY)
        objective_terms.append(min_distance)
    solutions = [] # TODO: sample initial solution from this set
    while len(solutions) < num_samples:
        terms = []
        for var in variables:
            for index, coord in enumerate(var):
                value = coord.X
                set_guess(coord, value, hard=True)
                delta = model.addVar(lb=-GRB.INFINITY, ub=GRB.INFINITY)
                model.addConstr(delta == coord - value)
                if norm == 1:
                    term = model.addVar(lb=0., ub=GRB.INFINITY)
                    model.addConstr(term == abs_(delta))
                elif norm == 2:
                    term = delta*delta
                else:
                    raise NotImplementedError(norm)
                terms.append(term) # TODO: scale
        distance = quicksum(terms)
        if closest:
            model.addConstr(min_distance <= distance)
        else:
            objective_terms.append(distance) # TODO: weight

        model.setObjective(quicksum(objective_terms), sense=GRB.MAXIMIZE) # MINIMIZE | MAXIMIZE
        model.optimize()
        print('# {} | objective: {:.3f} | cost: {:.3f} | runtime: {:.3f}'.format(
            len(solutions), model.ObjVal, objective.getValue(), elapsed_time(start_time))) # min_distance.X
        solution = [value_from_var(var) for var in variables]
        solutions.append(solution)
        yield solution

##################################################

def get_optimize_fn(regions, collisions=True, max_time=5., hard=False,
                    diagnostic='all', diagnose_cost=False, verbose=False):
    # https://www.gurobi.com/documentation/8.1/examples/diagnose_and_cope_with_inf.html
    # https://www.gurobi.com/documentation/8.1/examples/tsp_py.html#subsubsection:tsp.py
    # https://examples.xpress.fico.com/example.pl?id=Infeasible_python
    # https://www.fico.com/fico-xpress-optimization/docs/latest/examples/python/GUID-E77AC35C-9488-3F0A-98B4-7F5FD81AFF1D.html
    if not has_gurobi():
        raise ImportError('This generator requires Gurobi: http://www.gurobi.com/')
    from gurobipy import Model, GRB, quicksum, abs_

    min_x = min(x1 for x1, _ in regions.values())
    max_x = max(x2 for _, x2 in regions.values())
    min_y, max_y = 0, (max_x - min_x) / 2.
    lower = [min_x, min_y]
    upper = [max_x, max_y]

    # lower = 2*[-INF]
    # upper = 2*[+INF]
    # lower = upper = None

    def fn(outputs, facts, hint={}):
        # TODO: pass in the variables and constraint streams instead?
        # The true test is placing two blocks in a tight region obstructed by one
        constraint_indices = {i for i, term in enumerate(facts) if term[0] != MINIMIZE}
        positive, negative, costs = partition_facts(facts)
        #print('Parameters:', outputs)
        #print('Constraints:', positive + negative)
        if costs:
            print('Costs:', costs)

        # https://github.com/yijiangh/coop_assembly/blob/e52abef7c1cfb1d3e32691d163abc85dd77f27a2/src/coop_assembly/geometry_generation/caelan.py
        model = Model(name='TAMP')
        model.setParam(GRB.Param.OutputFlag, verbose)
        model.setParam(GRB.Param.TimeLimit, max_time)
        model.setParam(GRB.Param.Cutoff, GRB.INFINITY) # TODO: account for scaling
        #if num_solutions < INF:
        #    model.setParam(GRB.Param.SolutionLimit, num_solutions)

        # Limit how many solutions to collect
        #model.setParam(GRB.Param.PoolSolutions, 2)
        # Limit the search space by setting a gap for the worst possible solution that will be accepted
        #model.setParam(GRB.Param.PoolGap, 0.10) # PoolGapAbs
        # do a systematic search for the k-best solutions
        #model.setParam(GRB.Param.PoolSearchMode, 2) # 0 | 1 | 2
        # https://www.gurobi.com/documentation/9.1/examples/poolsearch_py.html#subsubsection:poolsearch.py

        ##########

        # TODO: remove anything that's just a domain condition?
        variable_indices = {}
        var_from_param = {}
        for index, fact in enumerate(facts):
            prefix, args = fact[0], fact[1:]
            if prefix == 'conf':
                param, = args
                if is_parameter(param):
                    var_from_param[param] = np_var(model, lower=lower, upper=upper)
            elif prefix == 'pose':
                _, param = args
                if is_parameter(param):
                    var_from_param[param] = np_var(model, lower=lower, upper=upper)
            elif prefix == 'grasp':  # TODO: iterate over combinations
                _, param = args
                if is_parameter(param):
                    var_from_param[param] = GRASP
            elif prefix == 'traj':
                raise NotImplementedError()
                #param, = args
                #if param not in var_from_id:
                #    var_from_id[id(param)] = [np_var(model), np_var(model)]
            else:
                continue
            variable_indices[index] = fact
        dimension = sum(len(var) for var in var_from_param.values())

        def get_var(p):
            return var_from_param[p] if is_parameter(p) else p

        ##########

        codimension = 0
        objective_terms = [] # TODO: could make a variable to impose a cost constraint
        constraint_from_name = {}
        for index, fact in enumerate(facts):
            prefix, args = fact[0], fact[1:]
            name = str(index)
            if prefix == 'kin':
                kinematics_constraint(model, name, *map(get_var, args))
                codimension += 2
            elif prefix in ('contain', 'contained'):
                contained_constraint(model, regions, name, *map(get_var, args))
                codimension += 1
            elif prefix == 'cfree' and collisions:
                # TODO: drop collision constraints until violated
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
                    objective_terms.extend(distance_cost(model, *map(get_var, args)))
                continue
            constraint_from_name[name] = fact
        model.update()

        ##########

        #linear_model = model
        linear_model = copy_model(model)
        #linear_model = Model(name='Linear TAMP')

        # TODO: prune linearly dependent constraints
        linear_constraints = {c for c in linear_model.getConstrs() if c.sense == GRB.EQUAL}
        codimension = len(linear_constraints)
        # TODO: account for v.LB == v.UB
        #linear_variables = {v for v in linear_model.getVars() if v.VType == GRB.CONTINUOUS}
        #print(vars_from_expr(linear_model.getObjective()))
        linear_variables = set()
        for c in linear_constraints:
            linear_variables.update(vars_from_expr(linear_model.getRow(c)))
        linear_variables = sorted(linear_variables, key=lambda v: v.VarName)
        dimension = len(linear_variables)

        print('{} variables (dim={}): {}'.format(len(variable_indices), dimension,
                                                 [facts[index] for index in sorted(variable_indices)]))
        nontrivial_indices = set(constraint_indices) - set(variable_indices) # TODO: rename
        print('{} constraints: (codim={}): {}'.format(len(nontrivial_indices), codimension,
                                                      [facts[index] for index in sorted(nontrivial_indices)]))

        # # https://en.wikipedia.org/wiki/Linear_subspace
        # # TODO: Equations for a subspace
        # #for c in model.getConstrs():
        # #    if c.sense != GRB.EQUAL:
        # #        model.remove(c)
        # variables = [model.getVarByName(v.VarName) for v in linear_variables]
        # lower_bound = np.array([v.LB for v in variables])
        # upper_bound = np.array([v.UB for v in variables])
        # center = (lower_bound + upper_bound) / 2.
        # extent = (upper_bound - lower_bound) / 2.
        # radius = np.linalg.norm(extent) # sphere
        #
        # point = radius*sample_sphere(dimension) + center
        # #point = center
        # basis = [sample_sphere_surface(dimension) for _ in range(codimension)]
        # #basis = [np.ones(dimension)]
        # multipliers = [unbounded_var(model) for _ in basis]
        # subspace_constraints = []
        # for i in range(dimension):
        #     combination = sum([m*b[i] for m, b in zip(multipliers, basis)])
        #     subspace_constraints.append(model.addConstr(variables[i] - point[i] == combination))
        # #for c in subspace_constraints:
        # #    model.remove(c)

        # TODO: generator version
        # for v in set(linear_model.getVars()) - linear_variables:
        #     linear_model.remove(v)
        # for c in set(linear_model.getConstrs()) - linear_constraints:
        #     linear_model.remove(c)
        # linear_model.setObjective(quicksum(sample_targets(linear_model, linear_variables)), sense=GRB.MINIMIZE)
        # linear_model.optimize()
        # for v in linear_variables: # Projection method
        #     set_value(model.getVarByName(v.VarName), v.X)

        ##########

        # TODO: normalize cost relative to the best cost for a trade-off
        # TODO: increasing bound on deterioration in quality
        weight = 0
        if weight > 0:
            primary_variables = {v for var in var_from_param.values() for v in var}
            objective_terms.extend(weight * term for term in sample_targets(model, primary_variables))
        model.setObjective(quicksum(objective_terms), sense=GRB.MINIMIZE) # (1-weight) * quicksum(objective_terms)

        for out, value in hint.items():
            for var, coord in zip(get_var(out), value):
                # https://www.gurobi.com/documentation/9.1/refman/varhintval.html#attr:VarHintVal
                set_guess(var, coord, hard=hard)
                #set_value(var, coord)

        ##########

        #m.write("file.lp")
        model.optimize()
        # https://www.gurobi.com/documentation/7.5/refman/optimization_status_codes.html
        #if model.status in (GRB.INFEASIBLE, GRB.INF_OR_UNBD, GRB.CUTOFF): # OPTIMAL | SUBOPTIMAL
        if model.SolCount == 0:
            if diagnostic is None:
                return OptimizerOutput()
            elif diagnostic == 'all':
                #infeasible = constraint_indices
                infeasible = nontrivial_indices
            elif diagnostic == 'deletion':
                infeasible = deletion_filter(model, constraint_indices)
            elif diagnostic == 'elastic':
                infeasible = elastic_filter(model, constraint_indices)
            elif diagnostic == 'gurobi':
                infeasible = compute_inconsistent(model)
            else:
                raise NotImplementedError(diagnostic)
            print('Inconsistent:', [facts[index] for index in sorted(infeasible)])
            return OptimizerOutput(infeasible=[infeasible])

            #expr.getValue() # TODO: store expressions and evaluate value
        # for c in model.getConstrs():
        #     print(c, c.Slack, c.RHS)
        #     print(c.__dict__)
        #     print(dir(c))

        ##########

        print('Solved: {} | Objective: {:.3f} | Solutions: {} | Status: {} | Runtime: {:.3f}'.format(
            True, model.ObjVal, model.SolCount, model.status, model.runtime))

        if costs and diagnose_cost:
            infeasible = deletion_filter(model, constraint_indices, max_objective=model.ObjVal - 1e-6)
        else:
            # TODO: propagate automatically to optimizer
            #infeasible = constraint_indices
            infeasible = nontrivial_indices
        print('Cost inconsistent:', [facts[index] for index in sorted(infeasible)])

        # variables = list(var_from_param.values())
        # for index, solution in enumerate(sample_solutions(model, variables, num_samples=15)):
        #    print(index, solution)

        assignment = tuple(value_from_var(get_var(out)) for out in outputs)
        return OptimizerOutput(assignments=[assignment], infeasible=[infeasible])
    return fn
    #return lambda outputs, facts, **kwargs: \
    #    identify_infeasibility(fn, outputs, facts, diagnose=False, **kwargs)

##################################################

def compute_inconsistent(model):
    from gurobipy import GRB
    # TODO: search over irreducible infeasible sets
    model.setObjective(0.0) # Makes a difference in performance
    model.setParam(GRB.Param.IISMethod, 1) # -1 | 0 | 1 | 2 | 3

    model.computeIIS()
    print('IIS is minimal\n' if model.IISMinimal else 'IIS is not minimal\n')
    #assert model.IISMinimal
    infeasible = {int(c.constrName) for c in model.getConstrs() if c.IISConstr}
    return infeasible

def enumerate_inconsistent(model):
    # TODO: exhaustively enumerate IIS
    raise NotImplementedError()

##################################################

def constraints_from_indices(model, indices):
    names = {str(i) for i in indices}
    return [c for c in model.getConstrs() if c.constrName in names]

def deletion_filter(original_model, indices, max_objective=INF):
    from gurobipy import GRB

    model = copy_model(original_model)
    if max_objective < INF:
        model.setParam(GRB.Param.Cutoff, max_objective)
    else:
        model = model.feasibility()

    # prune_constraints = set(model.getConstrs()) - set(constraints_from_indices(model, indices))
    # for c in prune_constraints:
    #     model.remove(c)
    model.optimize()
    #assert model.status in (GRB.INFEASIBLE, GRB.INF_OR_UNBD, GRB.CUTOFF)
    assert model.SolCount == 0

    inconsistent = set()
    for index in sorted(indices):
        temp_model = copy_model(model)
        constraints = constraints_from_indices(temp_model, {index})
        if not constraints:
            continue
        for c in constraints:
            temp_model.remove(c)
        temp_model.optimize()
        #if temp_model.status == GRB.INFEASIBLE:
        if temp_model.SolCount == 0:
            model = temp_model
        else:
            inconsistent.add(index)
    return inconsistent

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

def elastic_filter(original_model, indices, postproces=True):
    elastic = set(indices)
    inconsistent = set()
    while True:
        relaxed_model = copy_model(original_model)
        violated = relax_constraints(relaxed_model, elastic)
        if violated is None:
            break
        elastic -= violated
        inconsistent |= violated
    if postproces:
        inconsistent = deletion_filter(original_model, inconsistent)
    return inconsistent

##################################################

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

def identify_feasible_subsets(facts, model):
    # TODO: add facts corresponding to feasible subproblems
    # The trouble is that it's not clear which constraints would be useful to relax
    model.setObjective(0.0)
    model.computeIIS()
    print('IIS is minimal\n' if model.IISMinimal else 'IIS is not minimal\n')
    iss_constraints = {c.constrName for c in model.getConstrs() if c.IISConstr}
    #iss_constraints = compute_inconsistent(model)
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