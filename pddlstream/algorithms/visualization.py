from __future__ import print_function

import os

from pddlstream.algorithms.reorder import get_partial_orders
from pddlstream.language.constants import EQ, get_prefix, get_args, str_from_plan, is_parameter, \
    partition_facts
from pddlstream.language.conversion import str_from_fact, evaluation_from_fact
from pddlstream.language.function import FunctionResult
from pddlstream.language.object import OptimisticObject
from pddlstream.utils import clear_dir, ensure_dir, str_from_object, user_input, flatten

# https://www.graphviz.org/doc/info/

DEFAULT_EXTENSION = '.png' # png | pdf

PARAMETER_COLOR = 'LightGreen'
CONSTRAINT_COLOR = 'LightBlue'
NEGATED_COLOR = 'LightYellow'
COST_COLOR = 'LightSalmon'
STREAM_COLOR = 'LightSteelBlue'
FUNCTION_COLOR = 'LightCoral'

VISUALIZATIONS_DIR = 'visualizations/'
CONSTRAINT_NETWORK_DIR = os.path.join(VISUALIZATIONS_DIR, 'constraint_networks/')
STREAM_PLAN_DIR = os.path.join(VISUALIZATIONS_DIR, 'stream_plans/')
PLAN_LOG_FILE = os.path.join(VISUALIZATIONS_DIR, 'log.txt')
ITERATION_TEMPLATE = 'iteration_{}' + DEFAULT_EXTENSION
SYNTHESIZER_TEMPLATE = '{}_{}' + DEFAULT_EXTENSION

COMPLEXITY_ITERATION_TEMPLATE = '{}_{}' + DEFAULT_EXTENSION

##################################################

def has_pygraphviz():
    # TODO: networkx
    # https://github.com/caelan/pddlstream/blob/82ee5e363585d0af8ff9532ecc14641687d5b56b/examples/fault_tolerant/data_network/run.py#L189
    #import networkx
    #import graphviz
    #import pydot
    try:
        import pygraphviz
    except ImportError:
        return False
    return True

def reset_visualizations():
    clear_dir(VISUALIZATIONS_DIR)
    ensure_dir(CONSTRAINT_NETWORK_DIR)
    ensure_dir(STREAM_PLAN_DIR)

def load_plan_log():
    import json
    json_file = os.path.join(VISUALIZATIONS_DIR, 'log.json')
    plans = None
    if os.path.isfile(json_file):
        with open(json_file, 'r') as f:
            plans = json.load(f)
    else:
        with open(json_file, 'w') as f:
            json.dump({}, f)
            plans = []
    return json_file, plans

def log_failed_streams(name, args):
    json_file, plan_log = load_plan_log()
    if plan_log is not None and len(plan_log) > 0:
        streams = plan_log[-1]
        streams.append({
            'name': name, 'args': [str(n) for n in args]
        })
        plan_log[-1] = streams

        from pybullet_planning.pybullet_tools.logging import dump_json
        dump_json(plan_log, json_file, sort_dicts=False)

def log_actions(stream_plan, action_plan, iteration):
    json_file, plans = load_plan_log()

    actions = []
    for a in action_plan:
        actions.append({
            'name': a.name, 'args': [str(n) for n in a.args]
        })
    plans.append(actions)
    plans.append([])

    # with open(json_file, 'a+') as f:
        # json.dump(plans, f, indent=3)
    from pybullet_planning.pybullet_tools.logging import dump_json
    dump_json(plans, json_file, sort_dicts=False)


def log_plans(stream_plan, action_plan, iteration):
    # TODO: do this within the focused algorithm itself?
    from pddlstream.retired.synthesizer import decompose_stream_plan
    decomposed_plan = decompose_stream_plan(stream_plan)
    with open(PLAN_LOG_FILE, 'a+') as f:
        # f.write('Iteration: {}\n'
        #         'Component plan: {}\n'
        #         '\nStream plan:\n {}\n'
        #         '\nAction plan: {}\n\n'.format(
        #         iteration, decomposed_plan,
        #         '\n'.join([str(n) for n in stream_plan]), ## stream_plan, ## YANG: log plan
        #         str_from_plan(action_plan)))

        f.write('Action plan: \n{}\n\n'.format('\n'.join([str(n) for n in action_plan])))

def create_synthesizer_visualizations(result, iteration):
    from pddlstream.retired.synthesizer import decompose_result
    stream_plan = decompose_result(result)
    if len(stream_plan) <= 1:
        return
    # TODO: may overwrite another optimizer if both used on the same iteration
    filename = SYNTHESIZER_TEMPLATE.format(result.external.name, iteration)
    visualize_constraints(result.get_objectives(), os.path.join(CONSTRAINT_NETWORK_DIR, filename))
    visualize_stream_plan_bipartite(stream_plan, os.path.join(STREAM_PLAN_DIR, filename))

def create_visualizations(evaluations, stream_plan, iteration):
    # TODO: place it in the temp_dir?
    # TODO: decompose any joint streams
    for result in stream_plan:
        create_synthesizer_visualizations(result, iteration)
    filename = ITERATION_TEMPLATE.format(iteration)
    # filename = COMPLEXITY_ITERATION_TEMPLATE.format(complexity, iteration)
    # visualize_stream_plan(stream_plan, path)

    constraints = set() # TODO: approximates needed facts using produced ones
    for stream in stream_plan:
        constraints.update(filter(lambda f: evaluation_from_fact(f) not in evaluations, stream.get_certified()))
    print('Constraints:', str_from_object(constraints))
    g1 = visualize_constraints(constraints, os.path.join(CONSTRAINT_NETWORK_DIR, filename))
    g1.draw(os.path.join(CONSTRAINT_NETWORK_DIR, filename.replace('.png', '.dot')), prog='dot')

    from pddlstream.retired.synthesizer import decompose_stream_plan
    decomposed_plan = decompose_stream_plan(stream_plan)
    if len(decomposed_plan) != len(stream_plan):
        visualize_stream_plan(decompose_stream_plan(stream_plan), os.path.join(STREAM_PLAN_DIR, filename))

    #visualize_stream_plan_bipartite(stream_plan, os.path.join(STREAM_PLAN_DIR, 'fused_' + filename))
    g2 = visualize_stream_plan(stream_plan, os.path.join(STREAM_PLAN_DIR, 'fused_' + filename))
    g2.draw(os.path.join(STREAM_PLAN_DIR, filename.replace('.png', '.dot')), prog='dot')

##################################################

def visualize_constraints(constraints, filename='constraint_network'+DEFAULT_EXTENSION, use_functions=True):
    from pygraphviz import AGraph

    graph = AGraph(strict=True, directed=False)
    graph.node_attr['style'] = 'filled'
    #graph.node_attr['fontcolor'] = 'black'
    #graph.node_attr['fontsize'] = 12
    graph.node_attr['colorscheme'] = 'SVG'
    graph.edge_attr['colorscheme'] = 'SVG'
    #graph.graph_attr['rotate'] = 90
    #graph.node_attr['fixedsize'] = True
    graph.node_attr['width'] = 0
    graph.node_attr['height'] = 0.02 # Minimum height is 0.02
    graph.node_attr['margin'] = 0
    graph.graph_attr['rankdir'] = 'RL'
    graph.graph_attr['nodesep'] = 0.05
    graph.graph_attr['ranksep'] = 0.25
    #graph.graph_attr['pad'] = 0
    # splines="false";
    graph.graph_attr['outputMode'] = 'nodesfirst'
    graph.graph_attr['dpi'] = 300

    positive, negated, functions = partition_facts(constraints)
    for head in (positive + negated + functions):
        # TODO: prune values w/o free parameters?
        name = str_from_fact(head)
        if head in functions:
            if not use_functions:
                continue
            color = COST_COLOR
        elif head in negated:
            color = NEGATED_COLOR
        else:
            color = CONSTRAINT_COLOR
        graph.add_node(name, shape='box', color=color)
        for arg in get_args(head):
            if isinstance(arg, OptimisticObject) or is_parameter(arg):
                arg_name = str(arg)
                graph.add_node(arg_name, shape='circle', color=PARAMETER_COLOR)
                graph.add_edge(name, arg_name)

    graph.draw(filename, prog='dot') # neato | dot | twopi | circo | fdp | nop
    print('Saved', filename)
    return graph

##################################################

def display_image(filename):
    import matplotlib.pyplot as plt
    import matplotlib.image as mpimg
    img = mpimg.imread(filename)
    plt.imshow(img)
    plt.title(filename)
    plt.axis('off')
    plt.tight_layout()

    #plt.show()
    plt.draw()
    #plt.waitforbuttonpress(0)  # this will wait for indefinite time
    plt.pause(interval=1e-3)
    user_input()
    plt.close(plt.figure())

def visualize_stream_orders(orders, streams=[], filename='stream_orders'+DEFAULT_EXTENSION):
    from pygraphviz import AGraph
    graph = AGraph(strict=True, directed=True)
    graph.node_attr['style'] = 'filled'
    graph.node_attr['shape'] = 'box'
    graph.node_attr['color'] = STREAM_COLOR
    graph.node_attr['fontcolor'] = 'black'
    #graph.node_attr['fontsize'] = 12
    graph.node_attr['width'] = 0
    graph.node_attr['height'] = 0.02 # Minimum height is 0.02
    graph.node_attr['margin'] = 0
    graph.graph_attr['outputMode'] = 'nodesfirst'
    graph.graph_attr['dpi'] = 300

    def mystr(obj):  ## YANG: in place of str()
        return str(obj).replace(':',':\n').replace('->','\n->').replace(',','\n,')

    streams = set(streams) | set(flatten(orders))
    for stream in streams:
        graph.add_node(mystr(stream))
    for stream1, stream2 in orders:
        graph.add_edge(mystr(stream1), mystr(stream2))
    # TODO: could also print the raw values (or a lookup table)
    # https://stackoverflow.com/questions/3499056/making-a-legend-key-in-graphviz

    graph.draw(filename, prog='dot')
    print('Saved', filename)
    #display_image(filename)
    return graph

def visualize_stream_plan(stream_plan, filename='stream_plan'+DEFAULT_EXTENSION):
    return visualize_stream_orders(get_partial_orders(stream_plan), streams=stream_plan, filename=filename)

##################################################

def visualize_stream_plan_bipartite(stream_plan, filename='stream_plan'+DEFAULT_EXTENSION, use_functions=False):
    from pygraphviz import AGraph
    graph = AGraph(strict=True, directed=True)
    graph.node_attr['style'] = 'filled'
    graph.node_attr['shape'] = 'box'
    graph.node_attr['fontcolor'] = 'black'
    #graph.node_attr['fontsize'] = 12
    graph.node_attr['width'] = 0
    graph.node_attr['height'] = 0.02 # Minimum height is 0.02
    graph.node_attr['margin'] = 0
    #graph.graph_attr['rankdir'] = 'LR'
    graph.graph_attr['nodesep'] = 0.1
    graph.graph_attr['ranksep'] = 0.25
    graph.graph_attr['outputMode'] = 'nodesfirst'
    graph.graph_attr['dpi'] = 300
    # TODO: store these settings as a dictionary

    def add_fact(fact):
        head, color = (fact[1], COST_COLOR) if get_prefix(fact) == EQ else (fact, CONSTRAINT_COLOR)
        s_fact = str_from_fact(head)
        graph.add_node(s_fact, color=color)
        return s_fact

    def add_stream(stream):
        color = FUNCTION_COLOR if isinstance(stream, FunctionResult) else STREAM_COLOR
        s_stream = str(stream.instance) if isinstance(stream, FunctionResult) else str(stream)
        graph.add_node(s_stream, style='rounded,filled', color=color)
        # shape: oval, plaintext, polygon, rarrow, cds
        # style: rounded, filled, bold
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
    print('Saved', filename)
    return graph
    # graph.layout
    # https://pygraphviz.github.io/documentation/pygraphviz-1.3rc1/reference/agraph.html
    # https://pygraphviz.github.io/documentation/stable/reference/agraph.html#pygraphviz.AGraph.draw
