from pddlstream.object import OptimisticObject
from pddlstream.conversion import obj_from_pddl
from pddlstream.utils import input

def get_partial_orders(stream_plan):
    # TODO: only show the first atom achieved?
    partial_orders = set()
    for i, stream1 in enumerate(stream_plan):
        for stream2 in stream_plan[i+1:]: # Prevents circular
            if set(stream1.get_certified()) & set(stream2.get_certified()):
                partial_orders.add((stream1, stream2))
    return partial_orders

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

    graph.draw(filename, prog='dot')
    return graph


def open_pdf(filename):
    import subprocess
    # import os
    # import webbrowser
    subprocess.Popen('open {}'.format(filename), shell=True)
    # os.system(filename)
    # webbrowser.open(filename)
    input('Display?')
    # safe_remove(filename)

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
