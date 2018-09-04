import os

from pddlstream.algorithms.reorder import get_partial_orders
from pddlstream.language.constants import EQ, get_prefix, get_args, NOT, MINIMIZE
from pddlstream.language.conversion import evaluation_from_fact, str_from_fact
from pddlstream.language.function import FunctionResult
from pddlstream.language.object import OptimisticObject
from pddlstream.language.synthesizer import SynthStreamResult
from pddlstream.utils import clear_dir

# https://www.graphviz.org/doc/info/colors.html

PARAMETER_COLOR = 'LightGreen'
CONSTRAINT_COLOR = 'LightBlue'
NEGATED_COLOR = 'LightYellow'
COST_COLOR = 'LightSalmon'
STREAM_COLOR = 'LightSteelBlue'
FUNCTION_COLOR = 'LightCoral'

VISUALIZATIONS_DIR = 'visualizations/'
CONSTRAINT_NETWORK_DIR = os.path.join(VISUALIZATIONS_DIR, 'constraint_networks/')
STREAM_PLAN_DIR = os.path.join(VISUALIZATIONS_DIR, 'stream_plans/')
ITERATION_TEMPLATE = 'iteration_{}.pdf'
SYNTHESIZER_TEMPLATE = '{}_{}.pdf'

##################################################

def clear_visualizations():
    clear_dir(CONSTRAINT_NETWORK_DIR)
    clear_dir(STREAM_PLAN_DIR)

def get_optimistic_constraints(evaluations, stream_plan):
    # TODO: approximates needed facts using produced ones
    constraints = set()
    for stream in stream_plan:
        constraints.update(stream.get_certified())
    return set(filter(lambda f: evaluation_from_fact(f) not in evaluations, constraints))

def create_synthesizer_visualizations(result, num_iterations):
    stream_plan = result.decompose()
    filename = SYNTHESIZER_TEMPLATE.format(result.instance.external.name, num_iterations)
    #constraints = get_optimistic_constraints(evaluations, stream_plan)
    visualize_constraints(result.get_certified() + result.get_functions(),
                          os.path.join(CONSTRAINT_NETWORK_DIR, filename))
    visualize_stream_plan_bipartite(stream_plan,
                                    os.path.join(STREAM_PLAN_DIR, filename))

def create_visualizations(evaluations, stream_plan, num_iterations):
    # TODO: place it in the temp_dir?
    for result in stream_plan:
        if isinstance(result, SynthStreamResult):
            create_synthesizer_visualizations(result, num_iterations)
    filename = ITERATION_TEMPLATE.format(num_iterations)
    # visualize_stream_plan(stream_plan, path)
    constraints = get_optimistic_constraints(evaluations, stream_plan)
    visualize_constraints(constraints, os.path.join(CONSTRAINT_NETWORK_DIR, filename))
    visualize_stream_plan_bipartite(stream_plan, os.path.join(STREAM_PLAN_DIR, filename))

##################################################

def visualize_constraints(constraints, filename='constraint_network.pdf', use_functions=True):
    from pygraphviz import AGraph

    graph = AGraph(strict=True, directed=False)
    graph.node_attr['style'] = 'filled'
    graph.node_attr['shape'] = 'circle'
    graph.node_attr['fontcolor'] = 'black'
    graph.node_attr['colorscheme'] = 'SVG'
    graph.edge_attr['colorscheme'] = 'SVG'

    functions = set()
    negated = set()
    heads = set()
    for fact in constraints:
        prefix = get_prefix(fact)
        if prefix in (EQ, MINIMIZE):
            functions.add(fact[1])
        elif prefix == NOT:
            negated.add(fact[1])
        else:
            heads.add(fact)
    heads.update(functions)
    heads.update(negated)

    objects = {a for head in heads for a in get_args(head)}
    optimistic_objects = filter(lambda o: isinstance(o, OptimisticObject), objects)
    for opt_obj in optimistic_objects:
        graph.add_node(str(opt_obj), shape='circle', color=PARAMETER_COLOR)

    for head in heads:
        if not use_functions and (head in functions):
            continue
        # TODO: prune values w/o free parameters?
        name = str_from_fact(head)
        if head in functions:
            color = COST_COLOR
        elif head in negated:
            color = NEGATED_COLOR
        else:
            color = CONSTRAINT_COLOR
        graph.add_node(name, shape='box', color=color)
        for arg in get_args(head):
            if arg in optimistic_objects:
                graph.add_edge(name, str(arg))
    graph.draw(filename, prog='dot') # neato | dot | twopi | circo | fdp | nop
    return graph

##################################################

def visualize_stream_plan(stream_plan, filename='stream_plan.pdf'):
    from pygraphviz import AGraph
    graph = AGraph(strict=True, directed=True)
    graph.node_attr['style'] = 'filled'
    graph.node_attr['shape'] = 'box'
    graph.node_attr['color'] = STREAM_COLOR

    for stream in stream_plan:
        graph.add_node(str(stream))
    for stream1, stream2 in get_partial_orders(stream_plan):
        graph.add_edge(str(stream1), str(stream2))
    # TODO: could also print the raw values (or a lookup table)
    # https://stackoverflow.com/questions/3499056/making-a-legend-key-in-graphviz

    graph.draw(filename, prog='dot')
    return graph

##################################################

def visualize_stream_plan_bipartite(stream_plan, filename='stream_plan.pdf', use_functions=False):
    from pygraphviz import AGraph
    graph = AGraph(strict=True, directed=True)
    graph.node_attr['style'] = 'filled'
    graph.node_attr['shape'] = 'box'

    def add_fact(fact):
        head, color = (fact[1], COST_COLOR) if get_prefix(fact) == EQ else (fact, CONSTRAINT_COLOR)
        s_fact = str_from_fact(head)
        graph.add_node(s_fact, shape='box', color=color)
        return s_fact

    def add_stream(stream):
        color = FUNCTION_COLOR if isinstance(stream, FunctionResult) else STREAM_COLOR
        s_stream = str(stream.instance) if isinstance(stream, FunctionResult) else str(stream)
        graph.add_node(s_stream, shape='oval', color=color)
        return s_stream

    achieved_facts = set()
    for stream in stream_plan:
        if not use_functions and isinstance(stream, FunctionResult):
            continue
        s_stream = add_stream(stream)
        for fact in stream.instance.get_domain():
            if fact in achieved_facts:
                s_fact = add_fact(fact)
                graph.add_edge(s_fact, s_stream) # Add initial facts?
        #if not isinstance(stream, StreamResult):
        #    continue
        for fact in stream.get_certified():
            if fact not in achieved_facts: # Ensures DAG
                s_fact = add_fact(fact)
                graph.add_edge(s_stream, s_fact)
                achieved_facts.add(fact)
    graph.draw(filename, prog='dot')
    return graph
    # graph.layout
    # https://pygraphviz.github.io/documentation/pygraphviz-1.3rc1/reference/agraph.html
