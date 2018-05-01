from pddlstream.algorithm import get_partial_orders
from pddlstream.conversion import get_args
from pddlstream.object import OptimisticObject
from pddlstream.utils import str_from_tuple

def visualize_stream_plan(stream_plan, filename='stream_plan.pdf'):
    from pygraphviz import AGraph
    graph = AGraph(strict=True, directed=True)
    graph.node_attr['style'] = 'filled'
    graph.node_attr['shape'] = 'box'
    graph.node_attr['color'] = 'LightSalmon'

    for stream in stream_plan:
        graph.add_node(str(stream))
    for stream1, stream2 in get_partial_orders(stream_plan):
        graph.add_edge(str(stream1), str(stream2))
    # TODO: could also print the raw values (or a lookup table)

    graph.draw(filename, prog='dot')
    return graph

def visualize_stream_plan_bipartite(stream_plan, filename='stream_plan.pdf'):
    from pygraphviz import AGraph
    graph = AGraph(strict=True, directed=True)
    graph.node_attr['style'] = 'filled'
    graph.node_attr['shape'] = 'box'
    graph.node_attr['color'] = 'LightSalmon'

    achieved_facts = set()
    for stream in stream_plan:
        graph.add_node(str(stream), shape='oval', color='LightSalmon')
        for fact in stream.instance.get_domain():
            graph.add_node(str_from_tuple(fact), shape='box', color='LightBlue')
            graph.add_edge(str_from_tuple(fact), str(stream)) # Add initial facts?
        for fact in stream.get_certified():
            if fact not in achieved_facts: # Ensures DAG
                graph.add_node(str_from_tuple(fact), shape='box', color='LightBlue')
                graph.add_edge(str(stream), str_from_tuple(fact))
                achieved_facts.add(fact)
    graph.draw(filename, prog='dot')
    return graph

def visualize_constraints(constraints, filename='constraint_network.pdf'):
    from pygraphviz import AGraph

    graph = AGraph(strict=True, directed=False)
    graph.node_attr['style'] = 'filled'
    graph.node_attr['shape'] = 'circle'
    graph.node_attr['fontcolor'] = 'black'
    graph.node_attr['colorscheme'] = 'SVG'
    graph.edge_attr['colorscheme'] = 'SVG'

    objects = set()
    for constraint in constraints:
        objects.update(get_args(constraint))
    optimistic_objects = filter(lambda o: isinstance(o, OptimisticObject), objects)
    for opt_obj in optimistic_objects:
        graph.add_node(str(opt_obj), shape='circle', color='LightGreen')

    for constraint in constraints:
        # TODO: assert that is atom?
        name = str_from_tuple(constraint)
        graph.add_node(name, shape='box', color='LightBlue')
        for arg in get_args(constraint):
            if arg in optimistic_objects:
                graph.add_edge(name, str(arg))
    graph.draw(filename, prog='dot')
    return graph

"""
def visualize_constraints(constraints, filename='constraint_network.pdf'):
    from pygraphviz import AGraph

    graph = AGraph(strict=True, directed=False)
    graph.node_attr['style'] = 'filled'
    graph.node_attr['shape'] = 'circle'
    graph.node_attr['fontcolor'] = 'black'
    graph.node_attr['colorscheme'] = 'SVG'
    graph.edge_attr['colorscheme'] = 'SVG'

    objects = set()
    for constraint in constraints:
        objects.update(constraint.args)
    optimistic = filter(lambda n: n in OptimisticObject._obj_from_name, objects)
    for constant in optimistic:
        graph.add_node(str(constant), shape='circle', color='LightGreen')

    # TODO: print string for the value
    for constraint in constraints:
        # TODO: assert that is atom?
        #name = '\n'.join(str(arg) for arg in [constraints.predicate] + list(constraints.args))
        name = str(constraint)
        graph.add_node(name, shape='box', color='LightBlue')
        for arg in constraint.args:
            if arg in optimistic:
                graph.add_edge(name, str(arg))
    graph.draw(filename, prog='dot')
    return graph
"""