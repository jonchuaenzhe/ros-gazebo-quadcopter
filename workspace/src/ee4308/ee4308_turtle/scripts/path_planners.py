#!/usr/bin/env python
from math import sqrt
from numpy import inf
from turtle_constants import COST_MAP_FREE, COST_MAP_UNK, COST_FUNCTION
from line_of_sight import get_los_path

# constants
SQRT2 = sqrt(2)
REL = [[1, 0, 0], [1, 1, 0], [0, 1, 0], [-1, 1, 0], [-1, 0, 0], [-1, -1, 0], [0, -1, 0], [1, -1, 0]]

# global vars
num_j = 0
num_i = 0
num_k = 0
cost_map = None

parents = []
visited = []
g_costs = []
path_full = [] # in k
path_pts = [] # in (i,j)
open_list = []

def cost_euc(Di, Dj):
    return sqrt(Di*Di + Dj*Dj)
    
def cost_diag(Di, Dj):
    Di = abs(Di)
    Dj = abs(Dj)
    if Di > Dj:
        return Di - Dj + Dj*SQRT2
    else:
        return Dj - Di + Di*SQRT2

if COST_FUNCTION == "DIAGONAL":
    cost = cost_diag
else:
    cost = cost_euc

def is_free(cost_map_value):
    return cost_map_value == COST_MAP_FREE or cost_map_value == COST_MAP_UNK
    
def init_module(_num_i, _num_j):
    global num_i, num_j, num_k
    num_i = _num_i
    num_j = _num_j
    num_k = num_i*num_j
    for rel in REL:
        rel[2] = rel[0]*num_j + rel[1]
    reset_lists()
    
def reset_lists():
    global open_list, g_costs, visited, parents
    open_list = []
    # instantiate the parents matrix as a list of list of list of int
    parents = [-1 for k in xrange(num_k)]
    # instantiate the visited matrix as a list of list of boolean
    visited = [False for k in xrange(num_k)]
    # instantiate the g-cost matrix as a list of list of inf
    g_costs = [inf for k in xrange(num_k)]
    
def add_to_open(i, j, k, f, h):
    open_node = (i, j, k, f, h)    
    
    for n in xrange(len(open_list)):
        open_list_node = open_list[n]
        node_f = open_list_node[3]
        if node_f > f or (node_f == f and open_list_node[4] > h):
            # insert into current position n if cheaper than node in open list
            open_list.insert(n, open_node)
            return
            
    # either at the end (most expensive) or open list does not contain anything
    open_list.append(open_node)
    
    
def theta_star(source_i, source_j, target_i, target_j, cost_map):
    
    global path_full, path_pts
    path_full = []
    path_pts = []
    source_k = source_i*num_j + source_j 
    target_k = target_i*num_j + target_j 
    
    # return if start node is the goal node
    if source_k == target_k:
        path_pts.append((source_i, source_j))
        path_full.append(path_pts[0])
        return
    
    # set information for start node
    visited[source_k] = True
    g_costs[source_k] = 0.
    
    # find neighbors of start node and add them to open list
    for rel in REL:
        nb_i = rel[0] + source_i
        nb_j = rel[1] + source_j
        nb_k = rel[2] + source_k
        
        if nb_i >= 0 and nb_i < num_i and nb_j >= 0 and nb_j < num_j:
            # nb in map
            if is_free(cost_map[nb_k]):
                # nb not occupied and not inflation
                
                # get costs
                Di = target_i - nb_i
                Dj = target_j - nb_j
                nb_h = cost(Di, Dj)
                Di = nb_i - source_i
                Dj = nb_j - source_j
                nb_g = cost(Di, Dj)
                nb_f = nb_g + nb_h
                
                # assign g-cost
                g_costs[nb_k] = nb_g
                
                # assign parent
                parents[nb_k] = source_k
                
                # add to open_list
                add_to_open(nb_i, nb_j, nb_k, round(nb_f, 5), round(nb_h, 5))
    
    # begin search
    while open_list:
        # while open list contains something
        open_node = open_list.pop(0)
        cur_i = open_node[0]
        cur_j = open_node[1]
        cur_k = open_node[2]
        
        # check if it is visited (in closed list)
        if visited[cur_k]:
            continue # skip if visited (means a cheaper path was already expanded)
        # set to visited so we don't expand a more expensive route to this cell again
        visited[cur_k] = True
        
        # check if it is target / goal
        if cur_k == target_k:
            path_pts.append((cur_i, cur_j))
            path_full.append(cur_k)
            
            # iterate to start node
            par_k = parents[cur_k]
            while (par_k != -1):
                par_i = par_k // num_j
                par_j = par_k - par_i*num_j # or cur_k % num_j
                path_pts.append((par_i, par_j))
                
                # append to path_full by finding line betweeen turning points
                segment = get_los_path(cur_i, cur_j, cur_k, par_i, par_j, par_k, num_j)
                segment.pop(0) # remove the first element, which is cur_k
                for k in segment:
                    path_full.append(k)
                
                cur_k = par_k
                cur_i = par_i
                cur_j = par_j
                par_k = parents[cur_k]
            
            reset_lists()
            return
        
        # get current g cost
        cur_g = g_costs[cur_k]
        
        # get parent information
        par_k = parents[cur_k]
        par_i = par_k // num_j
        par_j = par_k - par_i*num_j
        par_g = g_costs[par_k]
        
        # loop over neighbors
        for rel in REL:
            nb_i = rel[0] + cur_i
            nb_j = rel[1] + cur_j
            nb_k = rel[2] + cur_k
            
            if nb_i >= 0 and nb_i < num_i and nb_j >= 0 and nb_j < num_j:
                # in map
                if is_free(cost_map[nb_k]):
                    # not occupied and not inflation
                    
                    # check if there is LOS to cur's parent
                    no_los_to_parent = False
                    for k in get_los_path(nb_i, nb_j, nb_k, par_i, par_j, par_k, num_j):
                        # for each cell in LOS,
                        if not is_free(cost_map[k]):
                            # occupied or inflated: no los to parent
                            no_los_to_parent = True
                            break
                            
                    if no_los_to_parent:
                        # no los to parent means g cost calculated from current cell
                        Di = nb_i - cur_i
                        Dj = nb_j - cur_j
                        nb_g = cost(Di, Dj) + cur_g
                    else:
                        # has los to parent means g cost calculated from parent of current cell
                        Di = nb_i - par_i
                        Dj = nb_j - par_j
                        nb_g = cost(Di, Dj) + par_g
                        
                    # compare g cost with neighbor's g cost, if any
                    if round(nb_g, 5) < round(g_costs[nb_k], 5):
                        # is cheapest so far
                        
                        # write the new g cost
                        g_costs[nb_k] = nb_g
                        
                        # write the new parent
                        if no_los_to_parent:
                            parents[nb_k] = cur_k
                        else:
                            parents[nb_k] = par_k
                        
                        # find the new f and h costs
                        Di = target_i - nb_i
                        Dj = target_j - nb_j
                        nb_h = cost(Di, Dj)
                        nb_f = nb_g + nb_h
                        
                        # add to open
                        add_to_open(nb_i, nb_j, nb_k, round(nb_f, 5), round(nb_h, 5))
    
    reset_lists() # clear up some memory
    return # no path at this point
    
def a_star_mod(source_i, source_j, target_i, target_j, cost_map, h_mult, source_i_f=None, source_j_f=None):
    
    source_k = source_i*num_j + source_j 
    if is_free(cost_map[source_k]):
        return (source_i, source_j)
    
    if source_i_f is None:
        source_i_f = source_i
        source_j_f = source_j
        
    add_to_open(source_i, source_j, source_k, 0., 0.) # add to open, source h_cost = f_cost

    while open_list:
        # while open list contains something
        open_node = open_list.pop(0)
        
        cur_k = open_node[2]        
        if is_free(cost_map[cur_k]):
            # print("Shifted from ({},{}) to ({},{})".format(source_i, source_j, cur_i, cur_j))
            reset_lists()
            return (cur_i, cur_j)
            
        cur_i = open_node[0]
        cur_j = open_node[1]
        cur_g = open_node[3] - open_node[4]
        
        for rel in REL:
            nb_k = rel[2] + cur_k
            if visited[nb_k] : # used differently here. to denote that cell is visited to avoid non-terminability
                continue
            visited[nb_k] = True
            
            nb_i = rel[0] + cur_i
            nb_j = rel[1] + cur_j
            
            if nb_i >= 0 and nb_i < num_i and nb_j >= 0 and nb_j < num_j:
                # in map
                
                nb_g = cost(nb_i - source_i_f, nb_j - source_j_f)
                
                # find the new f and h costs
                Di = target_i - nb_i
                Dj = target_j - nb_j
                nb_h = h_mult*cost(Di, Dj)
                nb_f = nb_g + nb_h
                
                # add to open
                add_to_open(nb_i, nb_j, nb_k, nb_f, nb_h)
    reset_lists()
    
def a_star(source_i, source_j, target_i, target_j, cost_map):
    print("AS", source_i, source_j, target_i, target_j)
    
    global path_full, path_pts
    path_full = []
    path_pts = []
    source_k = source_i*num_j + source_j 
    target_k = target_i*num_j + target_j 
    
    # return if start node is the goal node
    if source_k == target_k:
        path_pts.append((source_i, source_j))
        path_full.append(source_k)
        return
    
    # set information for start node
    g_costs[source_k] = 0. # cost calc is euclidean
    # calc cost
    Di = target_i - source_i
    Dj = target_j - source_j
    nb_h = round(cost(Di, Dj), 5) # reuse var for h cost
    add_to_open(source_i, source_j, source_k, nb_h, nb_h) # add to open, source h_cost = f_cost
    
    # begin search
    while open_list:
        # while open list contains something
        open_node = open_list.pop(0)
        cur_i = open_node[0]
        cur_j = open_node[1]
        cur_k = open_node[2]        
        
        # check if it is visited (in closed list)
        if visited[cur_k]:
            continue # skip if visited (means a cheaper path was already expanded)
        # set to visited so we don't expand a more expensive route to this cell again
        visited[cur_k] = True
        
        # check if it is target / goal
        if cur_k == target_k:
            path_pts.append((cur_i, cur_j))
            path_full.append(cur_k)
            
            # iterate to start node
            par_k = parents[cur_k]
            while (par_k != -1):
                path_full.append(par_k)
                par_i = par_k // num_j
                par_j = par_k - par_i*num_j # or cur_k % num_j
                
                # prepare for next loop
                cur_k = par_k
                cur_i = par_i
                cur_j = par_j
                par_k = parents[cur_k]
            
            # make the path of turning points
            if len(path_full) == 2:
                path_pts.append(path_full[1])
            else:
                par_k = path_full[1] # reuse
                prev_change_k = par_k - path_full[0]
                for n in xrange(2, len(path_full)):
                    cur_k = path_full[n]
                    change_k = cur_k - par_k
                    if change_k != prev_change_k: # turning point at par_k
                        par_i = par_k // num_j
                        par_j = par_k - par_i*num_j
                        path_pts.append((par_i, par_j))
                        
                    # prepare for next loop
                    prev_change_k = change_k
                    par_k = cur_k
                
                # append starting k
                path_pts.append((source_i, source_j))
                print(path_pts)
            reset_lists()
            return
        
        # get current g cost
        cur_g = g_costs[cur_k]
        
        # get parent information
        par_k = parents[cur_k]
        par_i = par_k // num_j
        par_j = par_k - par_i*num_j
        par_g = g_costs[par_k]
        
        # loop over neighbors
        for rel in REL:
            nb_i = rel[0] + cur_i
            nb_j = rel[1] + cur_j
            nb_k = rel[2] + cur_k
            
            if nb_i >= 0 and nb_i < num_i and nb_j >= 0 and nb_j < num_j:
                # in map
                if is_free(cost_map[nb_k]):
                    # not occupied and not inflation
                    
                    if rel[0] == 0 or rel[1] == 0:
                        # cardinal
                        nb_g = cur_g + 1.
                    else:
                        # ordinal
                        nb_g = cur_g + SQRT2
                        
                    # compare g cost with neighbor's g cost, if any
                    if round(nb_g, 5) < round(g_costs[nb_k], 5):
                        # is cheapest so far
                        
                        # write the new g cost
                        g_costs[nb_k] = nb_g
                        
                        # write the new parent
                        parents[nb_k] = cur_k                        
                        
                        # find the new f and h costs
                        Di = target_i - nb_i
                        Dj = target_j - nb_j
                        nb_h = cost(Di, Dj)
                        nb_f = nb_g + nb_h
                        
                        # add to open
                        add_to_open(nb_i, nb_j, nb_k, round(nb_f, 5), round(nb_h, 5))
    
    reset_lists() # clear up some memory
    return # no path at this point
