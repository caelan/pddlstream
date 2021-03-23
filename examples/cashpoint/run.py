#!/usr/bin/env python2.7

from __future__ import print_function

from random import uniform

from pddlstream.algorithms.search import solve_from_pddl
from pddlstream.utils import read_pddl, irange, INF, randomize, Profiler
from pddlstream.language.constants import get_length, PDDLProblem, print_solution, Exists, And, NOT, MINIMIZE, is_parameter
from pddlstream.language.generator import from_test, from_gen, from_fn, from_sampler, from_list_fn
from pddlstream.algorithms.incremental import solve_incremental
from pddlstream.algorithms.focused import solve_focused
from examples.continuous_tamp.optimizer.optimizer import has_gurobi
from pddlstream.language.optimizer import OptimizerOutput
from pddlstream.language.stream import StreamInfo, PartialInputs

# https://github.com/Emresav/ECAI16Domains/blob/master/cashpoint/domain0.pddl
# https://github.com/Emresav/ECAI16Domains/blob/master/cashpoint/p0.pddl
# https://github.com/Emresav/ECAI16Domains/tree/master/experimental%20results

# TODO: rocket and car domains

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

##################################################

def get_problem(optimize=True):
    initial_atm = 30
    initial_person = 2

    min_take = 1
    #max_take = 10
    max_take = initial_atm
    target = 50 # 1 | 3 | 50
    num_atms = 2

    domain_pddl = read_pddl(__file__, 'domain0.pddl')
    constant_map = {
        #'@min': min_take,
        #'@max': max_take,
        '@amount': target,
    }

    if optimize:
        stream_pddl = read_pddl(__file__, 'optimizer.pddl')
    else:
        stream_pddl = read_pddl(__file__, 'stream.pddl')

    # TODO: store history of withdraws to ensure not taking too much
    #stream_pddl = None
    stream_map = {
        #'s-cash': from_gen((c,) for c in randomize(irange(min_take, max_take + 1))),
        #'s-cash': from_gen((c,) for c in reversed(list(irange(min_take, max_take+1)))),
        's-cash': from_sampler(lambda: (round(uniform(min_take, max_take), 2),)),
        't-ge': from_test(lambda c1, c2: c1 >= c2),
        'add': from_fn(lambda c1, c2: (c1 + c2,)),
        'subtract': from_fn(lambda c3, c2: (c3 - c2,) if c3 - c2 >= 0 else None),
        'withdraw': from_fn(lambda wc, pc1, mc1: (pc1 + wc, mc1 - wc) if mc1 - wc >= 0 else None),
        'withdrawcost': lambda c: c, # TODO: withdraw fee
        'gurobi': create_optimizer(min_take, max_take),
    }

    person = 'Emre'
    #initial_people = {'Emre': initial_person}
    #initial_atms = {'Emre': initial_person}
    #amounts = [min_take, max_take, target, initial_atm, initial_person]

    init = [
        ('person', person),
        ('pcash', initial_person),
        ('pcash', target),
        ('tcash', target),
        ('inpocket', person, initial_person),
    ] # + [('cash', amount) for amount in amounts]

    for num in range(num_atms):
        name = 'atm{}'.format(num+1)
        init.extend([
            ('machine', name),
            ('mcash', initial_atm),
            ('maxwithdraw', name, initial_atm),
        ])

    #goal = Exists(['?c1'], And(('person', person), ('ge', '?c1', target),
    #                           ('inpocket', person, '?c1')))
    goal = ('finished',) # TODO: focused bug when goal initially satisfied

    return PDDLProblem(domain_pddl, constant_map, stream_pddl, stream_map, init, goal)

##################################################

def solve_pddlstream(focused=True, planner='max-astar', unit_costs=False):
    problem = get_problem()
    print(problem.constant_map)
    print(problem.init)
    print(problem.goal)

    stream_info = {
        't-ge': StreamInfo(eager=True),
        'withdraw': StreamInfo(opt_gen_fn=PartialInputs(unique=True)),
    }
    with Profiler(field='cumtime'):
        if focused:
            solution = solve_focused(problem, stream_info=stream_info,
                                     planner=planner, unit_costs=unit_costs,
                                     initial_complexity=3,
                                     clean=False, debug=True, verbose=True)
        else:
            solution = solve_incremental(problem, planner=planner, unit_costs=unit_costs,
                                         clean=False, debug=False, verbose=True)
    print_solution(solution)
    plan, cost, certificate = solution
    print('Certificate:', certificate.preimage_facts)

##################################################

def solve_pddl():
    domain_pddl = read_pddl(__file__, 'domain0.pddl')
    problem_pddl = read_pddl(__file__, 'problem0.pddl')

    plan, cost = solve_from_pddl(domain_pddl, problem_pddl)
    solved = plan is not None
    print('Solved: {}'.format(solved))
    print('Cost: {}'.format(cost))
    print('Length: {}'.format(get_length(plan)))
    if not solved:
        return
    for i, action in enumerate(plan):
        print('{}) {}'.format(i+1, ' '.join(map(str, action))))

def main():
    #solve_pddl()
    solve_pddlstream()

if __name__ == '__main__':
    main()
