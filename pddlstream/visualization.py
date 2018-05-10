import os

from pddlstream.conversion import get_args, EQ, get_prefix, fact_from_evaluation
from pddlstream.function import FunctionResult
from pddlstream.object import OptimisticObject
from pddlstream.reorder import get_partial_orders
from pddlstream.utils import str_from_tuple, clear_dir

# https://www.graphviz.org/doc/info/colors.html

PARAMETER_COLOR = 'LightGreen'
CONSTRAINT_COLOR = 'LightBlue'
COST_COLOR = 'LightSalmon'
STREAM_COLOR = 'LightSteelBlue'
FUNCTION_COLOR = 'LightCoral'

VISUALIZATIONS_DIR = 'visualizations/'
CONSTRAINT_NETWORK_DIR = os.path.join(VISUALIZATIONS_DIR, 'constraint_networks/')
STREAM_PLAN_DIR = os.path.join(VISUALIZATIONS_DIR, 'stream_plans/')
ITERATION_TEMPLATE = 'iteration_{}.pdf'

##################################################

def clear_visualizations():
    clear_dir(CONSTRAINT_NETWORK_DIR)
    clear_dir(STREAM_PLAN_DIR)


def create_visualizations(evaluations, stream_plan, num_iterations):
    # TODO: place it in the temp_dir?
    filename = ITERATION_TEMPLATE.format(num_iterations)
    # visualize_stream_plan(stream_plan, path)
    visualize_constraints(get_optimistic_constraints(evaluations, stream_plan),
                          os.path.join(CONSTRAINT_NETWORK_DIR, filename))
    visualize_stream_plan_bipartite(stream_plan,
                                    os.path.join(STREAM_PLAN_DIR, filename))

##################################################

def visualize_constraints(constraints, filename='constraint_network.pdf'):
    from pygraphviz import AGraph

    graph = AGraph(strict=True, directed=False)
    graph.node_attr['style'] = 'filled'
    graph.node_attr['shape'] = 'circle'
    graph.node_attr['fontcolor'] = 'black'
    graph.node_attr['colorscheme'] = 'SVG'
    graph.edge_attr['colorscheme'] = 'SVG'

    functions = set()
    heads = set()
    for fact in constraints:
        if get_prefix(fact) == EQ:
            functions.add(fact[1])
        else:
            heads.add(fact)
    heads.update(functions)

    objects = {a for head in heads for a in get_args(head)}
    optimistic_objects = filter(lambda o: isinstance(o, OptimisticObject), objects)
    for opt_obj in optimistic_objects:
        graph.add_node(str(opt_obj), shape='circle', color=PARAMETER_COLOR)

    for head in heads:
        # TODO: prune values w/o free parameters?
        name = str_from_tuple(head)
        color = COST_COLOR if head in functions else CONSTRAINT_COLOR
        graph.add_node(name, shape='box', color=color)
        for arg in get_args(head):
            if arg in optimistic_objects:
                graph.add_edge(name, str(arg))
    graph.draw(filename, prog='dot')
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

def visualize_stream_plan_bipartite(stream_plan, filename='stream_plan.pdf'):
    from pygraphviz import AGraph
    graph = AGraph(strict=True, directed=True)
    graph.node_attr['style'] = 'filled'
    graph.node_attr['shape'] = 'box'

    def add_fact(fact):
        head, color = (fact[1], COST_COLOR) if get_prefix(fact) == EQ else (fact, CONSTRAINT_COLOR)
        s_fact = str_from_tuple(head)
        graph.add_node(s_fact, shape='box', color=color)
        return s_fact

    def add_stream(stream):
        color = FUNCTION_COLOR if isinstance(stream, FunctionResult) else STREAM_COLOR
        s_stream = str(stream.instance) if isinstance(stream, FunctionResult) else str(stream)
        graph.add_node(s_stream, shape='oval', color=color)
        return s_stream

    achieved_facts = set()
    for stream in stream_plan:
        s_stream = add_stream(stream)
        for fact in stream.instance.get_domain():
            if fact in achieved_facts:
                s_fact = add_fact(fact)
                graph.add_edge(s_fact, s_stream) # Add initial facts?
        for fact in stream.get_certified():
            if fact not in achieved_facts: # Ensures DAG
                s_fact = add_fact(fact)
                graph.add_edge(s_stream, s_fact)
                achieved_facts.add(fact)
    graph.draw(filename, prog='dot')
    return graph


def get_optimistic_constraints(evaluations, stream_plan):
    # TODO: approximates needed facts using produced ones
    constraints = set()
    for stream in stream_plan:
        constraints.update(stream.get_certified())
    return constraints - set(map(fact_from_evaluation, evaluations))