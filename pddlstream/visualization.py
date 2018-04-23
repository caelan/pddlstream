from pddlstream.object import OptimisticObject
from pddlstream.conversion import obj_from_pddl

# def visualize_order(streams, edges, filename='directed_graph.pdf'):
#     from pygraphviz import AGraph
#     import subprocess
#     graph = AGraph(strict=True, directed=True)
#     graph.node_attr['style'] = 'filled'
#     graph.node_attr['shape'] = 'box'
#     graph.node_attr['color'] = 'LightSalmon'
#
#     def get_name(stream):
#         return '\n'.join([stream.cond_stream.name,
#                           ' '.join(str(inp) for inp in stream.inputs),
#                           ' '.join(str(inp) for inp in stream.abs_outputs)])
#
#     for stream in streams:
#         graph.add_node(get_name(stream))
#     for stream1, stream2 in edges:
#         graph.add_edge(get_name(stream1), get_name(stream2))
#
#     graph.draw(filename, prog='dot')
#     subprocess.Popen('open %s' % filename, shell=True)
#     raw_input('Display?')
#     return graph


def visualize_constraints(constraints, filename='graph.pdf'):
    from pygraphviz import AGraph
    import subprocess

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

    for constraint in constraints:
        # TODO: assert that is atom?



        #name = '\n'.join(str(arg) for arg in [constraints.predicate] + list(constraints.args))
        name = str(constraint)
        graph.add_node(name, shape='box', color='LightBlue')
        for arg in constraint.args:
            if arg in optimistic:
                graph.add_edge(name, str(arg))

    # import os
    # import webbrowser
    graph.draw(filename, prog='dot')
    subprocess.Popen('open {}'.format(filename), shell=True)
    # os.system(filename)
    # webbrowser.open(filename)
    raw_input('Display?')
    # safe_remove(filename)
    return graph
