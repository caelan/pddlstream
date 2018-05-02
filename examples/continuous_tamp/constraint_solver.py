import numpy as np

from examples.continuous_tamp.viewer import SUCTION_HEIGHT

BLOCK_WIDTH = 2
BLOCK_HEIGHT = BLOCK_WIDTH
GRASP = -np.array([0, BLOCK_HEIGHT + SUCTION_HEIGHT/2]) # TODO: side grasps


def get_constraint_solver(regions, max_time=5, verbose=False):
    # import cvxopt
    # import scipy.optimize.linprog
    # import mosek # https://www.mosek.com/
    from gurobipy import Model, GRB, quicksum
    def constraint_solver(facts):
        m = Model(name='TAMP')
        m.setParam(GRB.Param.OutputFlag, verbose)
        m.setParam(GRB.Param.TimeLimit, max_time)

        variable_from_id = {}
        for fact in facts:
            name, args = fact[0], fact[1:]
            if name == 'conf':
                param, = args
                if param not in variable_from_id:
                    variable_from_id[id(param)] = [m.addVar(lb=-GRB.INFINITY) for _ in range(2)]
            elif name == 'pose':
                _, param = args
                if param not in variable_from_id:
                    variable_from_id[id(param)] = [m.addVar(lb=-GRB.INFINITY) for _ in range(2)]

        objective_terms = []
        for fact in facts:
            name, args = fact[0], fact[1:]
            if name == 'kin':
                _, q, p = args
                conf = variable_from_id.get(id(q), q)
                pose = variable_from_id.get(id(p), p)
                for i in range(len(conf)):
                    m.addConstr(conf[i] + GRASP[i] == pose[i])
            elif name == 'contained':
                _, p, r = args
                px, py = variable_from_id.get(id(p), p)
                x1, x2 = regions[r]
                m.addConstr(x1 <= px - BLOCK_WIDTH/2)
                m.addConstr(px + BLOCK_WIDTH/2 <= x2)
                m.addConstr(py == 0)
            elif name == 'cfree':
                p1, p2 = args
                p1x, _ = variable_from_id.get(id(p1), p1)
                p2x, _ = variable_from_id.get(id(p2), p2)
                dist = m.addVar(lb=-GRB.INFINITY)
                abs_dist = m.addVar(lb=-GRB.INFINITY)
                m.addConstr(dist == p2x - p1x)
                m.addGenConstrAbs(abs_dist, dist) # abs_
                m.addConstr(BLOCK_WIDTH <= abs_dist)
            elif name == '=':
                fact = args[0]
                name, args = fact[0], fact[1:]
                if name == 'distance':
                    q1, q2 = args
                    conf1 = variable_from_id.get(id(q1), q1)
                    conf2 = variable_from_id.get(id(q2), q2)
                    for i in range(len(conf1)):
                        delta = conf2[i] - conf1[i]
                        objective_terms.append(delta*delta)

        m.setObjective(quicksum(objective_terms), sense=GRB.MINIMIZE)
        m.optimize()
        if m.status in (GRB.INFEASIBLE, GRB.INF_OR_UNBD):
            # https://www.gurobi.com/documentation/7.5/refman/optimization_status_codes.html
            return []
        value_from_id = {key: np.array([v.X for v in vars]) for key, vars in variable_from_id.items()}
        atoms = []
        for fact in facts:
            name, args = fact[0], fact[1:]
            if name in ('conf', 'pose', 'kin', 'contained', 'cfree'):
                new_fact = (name,) + tuple(value_from_id.get(id(a), a) for a in args)
                atoms.append(new_fact)
        return atoms
    return constraint_solver