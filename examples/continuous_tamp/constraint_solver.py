import numpy as np

from examples.continuous_tamp.primitives import BLOCK_WIDTH, GRASP, sample_region, plan_motion

MIN_CLEARANCE = 1e-3 # 0 | 1e-3

def get_constraint_solver(regions, max_time=5, verbose=False):
    # import cvxopt
    # import scipy.optimize.linprog
    # import mosek # https://www.mosek.com/
    from gurobipy import Model, GRB, quicksum, Var
    def constraint_solver(facts):
        m = Model(name='TAMP')
        m.setParam(GRB.Param.OutputFlag, verbose)
        m.setParam(GRB.Param.TimeLimit, max_time)

        variable_from_id = {}
        def unbounded_var():
            return m.addVar(lb=-GRB.INFINITY)
        def get_var(param):
            return variable_from_id.get(id(param), param)

        for fact in facts:
            name, args = fact[0], fact[1:]
            if name == 'conf':
                param, = args
                if param not in variable_from_id:
                    variable_from_id[id(param)] = np.array([unbounded_var() for _ in range(2)])
            elif name == 'pose':
                _, param = args
                if param not in variable_from_id:
                    variable_from_id[id(param)] = np.array([unbounded_var() for _ in range(2)])
            elif name == 'traj':
                param, = args
                if param not in variable_from_id:
                    q1 = np.array([unbounded_var() for _ in range(2)])
                    q2 = np.array([unbounded_var() for _ in range(2)])
                    variable_from_id[id(param)] = [q1, q2]

        objective_terms = []
        for fact in facts:
            name, args = fact[0], fact[1:]
            if name == 'kin':
                _, q, p = map(get_var, args)
                for i in range(len(q)):
                    m.addConstr(q[i] + GRASP[i] == p[i])
            elif name == 'contained':
                _, p, r = args
                px, py = variable_from_id.get(id(p), p)
                x1, x2 = regions[r]
                m.addConstr(x1 <= px - BLOCK_WIDTH / 2)
                m.addConstr(px + BLOCK_WIDTH / 2 <= x2)
                m.addConstr(py == 0)
            elif name in ('cfree', 'collision'):
                p1, p2 = map(get_var, args)
                dist = unbounded_var()
                abs_dist = unbounded_var()
                m.addConstr(dist == p2[0] - p1[0])
                m.addGenConstrAbs(abs_dist, dist) # abs_
                m.addConstr(BLOCK_WIDTH + MIN_CLEARANCE <= abs_dist)
            elif name == 'motion':
                q1, t, q2 = map(get_var, args)
                for i in range(len(q1)):
                    m.addConstr(t[0][i] == q1[i])
                for i in range(len(q2)):
                    m.addConstr(t[1][i] == q2[i])
            elif name == '=':
                fact = args[0]
                name, args = fact[0], fact[1:]
                if name == 'distance':
                    q1, q2 = map(get_var, args)
                    for i in range(len(q1)):
                        delta = q2[i] - q1[i]
                        objective_terms.append(delta*delta)

        m.setObjective(quicksum(objective_terms), sense=GRB.MINIMIZE)
        m.optimize()
        if m.status in (GRB.INFEASIBLE, GRB.INF_OR_UNBD):
            # https://www.gurobi.com/documentation/7.5/refman/optimization_status_codes.html
            return []

        def value_from_var(vars):
            if isinstance(vars, Var):
                return vars.X
            new_vars = list(map(value_from_var, vars))
            if isinstance(vars, np.ndarray):
                return np.array(new_vars)
            return new_vars

        # TODO: cost on endpoints and subtract from total cost

        value_from_id = {key:  value_from_var(vars) for key, vars in variable_from_id.items()}
        atoms = []
        for fact in facts:
            name, args = fact[0], fact[1:]
            if name in ('conf', 'pose', 'kin', 'contained', 'cfree', 'motion'):
                new_args = tuple(value_from_id.get(id(a), a) for a in args)
                new_fact = (name,) + new_args
                atoms.append(new_fact)
        return atoms
    return constraint_solver

##################################################

def cfree_motion_fn(outputs, certified):
    assert(len(outputs) == 1)
    q0, q1 = None, None
    placed = {}
    for fact in certified:
        if fact[0] == 'motion':
            q0, _, q1 = fact[1:]
        if fact[0] == 'not':
            _, b, p =  fact[1][1:]
            placed[b] = p
    return plan_motion(q0, q1)


def get_cfree_pose_fn(regions):
    def fn(outputs, certified):
        print(outputs, certified)
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


def get_optimize_fn(regions):
    def fn(outputs, certified):
        print(outputs, certified)
        raw_input('awefawef')
    return fn