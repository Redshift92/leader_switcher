# import networkx as nx
from scipy import weave

import rospkg
from os import path as os_path
headers_path = os_path.join(rospkg.RosPack().get_path('leader_switcher'), 'src', 'leader_switcher')

def init_graph(free_space, obstacles, start_positions, goal_positions):
    return weave.inline("printf(\"c\");return_val = init_graph(free_space, obstacles, start_positions, goal_positions);",
                        ['free_space', 'obstacles', 'start_positions', 'goal_positions'],
                        headers=['"'+ headers_path + '/csrc/solver.h"'])

def get_graph_edges():
    return weave.inline('return_val = get_graph_edges();printf(\"m\");', headers=['"'+ headers_path + '/csrc/solver.h"'])

def run():
    pass

def get_states_sequence():
    pass
