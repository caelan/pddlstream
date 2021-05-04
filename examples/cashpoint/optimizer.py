from examples.continuous_tamp.optimizer.optimizer import has_gurobi
from pddlstream.language.constants import is_parameter, MINIMIZE, NOT
from pddlstream.language.generator import from_list_fn
from pddlstream.language.optimizer import OptimizerOutput
from pddlstream.utils import INF


def create_optimizer(min_take=0, max_take=INF, max_wallet=INF, max_time=5, integer=True, diagnose=True, verbose=True):
    # https://www.gurobi.com/documentation/8.1/examples/diagnose_and_cope_with_inf.html
    # https://www.gurobi.com/documentation/8.1/examples/tsp_py.html#subsubsection:tsp.py
    # https://examples.xpress.fico.com/example.pl?id=Infeasible_python
    # https://www.fico.com/fico-xpress-optimization/docs/latest/examples/python/GUID-E77AC35C-9488-3F0A-98B4-7F5FD81AFF1D.html
    if not has_gurobi():
        raise ImportError('This generator requires Gurobi: http://www.gurobi.com/')
    from gurobipy import Model, GRB, quicksum, GurobiError

    def fn(outputs, facts, hint={}):
        print(outputs, facts)

        model = Model(name='TAMP')
        model.setParam(GRB.Param.OutputFlag, verbose)
        if max_time < INF:
            model.setParam(GRB.Param.TimeLimit, max_time)

        var_from_param = {}
        for fact in facts:
            prefix, args = fact[0], fact[1:]
            if prefix in ['wcash', 'pcash', 'mcash']:
                cash, = args
                if is_parameter(cash):
                    # TODO: scale by 100 for cents
                    var_from_param[cash] = model.addVar(
                        lb=0, ub=GRB.INFINITY, vtype=GRB.INTEGER if integer else GRB.CONTINUOUS)
                    if prefix == 'wcash':
                        model.addConstr(var_from_param[cash] >= min_take)
                        if max_take < INF:
                            model.addConstr(var_from_param[cash] <= max_take)
                    if (prefix == 'pcash') and (max_wallet < INF):
                        model.addConstr(var_from_param[cash] <= max_wallet)
                    if prefix == 'mcash':
                        # min_balance >= 0
                        pass

        get_var = lambda p: var_from_param[p] if is_parameter(p) else p # var_from_param.get(p, p)

        objective_terms = []
        for index, fact in enumerate(facts):
            name = str(index)
            if fact[0] == MINIMIZE:
                fact = fact[1]
                func, args = fact[0], map(get_var, fact[1:])
                if func == 'withdrawcost':
                    cash, = args
                    objective_terms.append(cash)
            elif fact[0] == NOT:
                fact = fact[1]
                predicate, args = fact[0], map(get_var, fact[1:])
            else:
                prefix, args = fact[0], map(get_var, fact[1:])
                if prefix == 'ge':
                    cash1, cash2 = args
                    model.addConstr(cash1 >= cash2, name=name)
                elif prefix == 'withdraw':
                    wcash, pcash1, pcash2, mcash1, mcash2 = args
                    model.addConstr(pcash1 + wcash == pcash2, name=name)
                    model.addConstr(mcash1 - wcash == mcash2, name=name)
        model.setObjective(quicksum(objective_terms), sense=GRB.MINIMIZE)

        try:
            model.optimize()
        except GurobiError as e:
            raise e

        objective = 0
        if objective_terms:
            objective = INF if model.status == GRB.INFEASIBLE else model.objVal
        print('Objective: {:.3f} | Solutions: {} | Status: {}'.format(objective, model.solCount, model.status))

        # https://www.gurobi.com/documentation/9.0/refman/optimization_status_codes.html
        if not model.solCount: # GRB.INFEASIBLE | GRB.INF_OR_UNBD | OPTIMAL | SUBOPTIMAL | UNBOUNDED
            return OptimizerOutput()
        assignment = tuple(get_var(out).x for out in outputs)
        return OptimizerOutput(assignments=[assignment])
    return from_list_fn(fn)