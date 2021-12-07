import os
from queue import PriorityQueue
from typing import Any, Dict, List, Tuple
from enum import Enum
import gurobipy as gp

from gurobipy import GRB
import problem_generator as pg
import networkx as nx
import threading
import collections

class Direction(Enum):
    WAIT=0
    EAST=1
    SOUTH=2
    WEST=3
    NORTH=4


"""
Helper function: check if a tuple is in range [t_min, t_max).

Args:
    t (Tuple): tuple to check
    t_min (Tuple): lower bound
    t_max (Tuple): upper bound

Returns:
    bool: True if t in range, False otherwise
"""
def tuple_in_range(t: Tuple, t_min: Tuple, t_max: Tuple) -> bool:
  
    for i in range(len(t)):
        if t[i] >= t_min[i] and t[i] < t_max[i]:
            continue
        else:
            return False
    return True



def dict_insert_helper(dictionary: Dict, key: Any, value: Any):
    """ Helper function to insert the value in to a list returned from dict[key]. 
    The function first checks if the input dictionary has the key. 
    If not, a list is constructed at dict[key]; else, append the value to dict[key].

    Args:
        dictionary (Dict): any dictionary
        key (Any): key in dictionary
        value (Any): value to insert
    """
    if key in dictionary:
        dictionary[key].append(value)
    else:
        dictionary[key] = [value]




def manhattan_distance(v1: Tuple[int, int], v2: Tuple[int, int]) -> int:

    return abs(v1[0] - v2[0]) + abs(v1[1] - v2[1])


def check_edge_type(v1: Tuple[int, int], v2: Tuple[int, int]) -> str:
    """ Helper function to check whether an edge is horizontal or vertical.

    Args:
        v1 (Tuple[int, int]): a vertex coordinate
        v2 (Tuple[int, int]): a vertex coordinate

    Returns:
        str: horizontal or vertical indicator
    """
    if v1[0] == v2[0] and v1[1] + 1 == v2[1]:
        return "vertical"
    elif v1[0] == v2[0] and v1[1] - 1 == v2[1]:
        return "vertical"
    elif v1[0] + 1 == v2[0] and v1[1] == v2[1]:
        return "horizontal"
    elif v1[0] - 1 == v2[0] and v1[1] == v2[1]:
        return "horizontal"
    else:
        return None


def shortest_path_length(graph: nx.Graph, start: Tuple[int, int],
                         target: Tuple[int, int]) -> int:
    """ Given a graph, a start vertex and a target vertex, return the shortest path length between the two vertices using an A* search algorithm.

    Args:
        graph (Graph): graph
        start (Tuple[int, int]): start vertex
        target (Tuple[int, int]): target vertex

    Raises:
        ValueError: cannot find a path between the two vertices

    Returns:
        int: the shortest path length between the two vertices
    """
    # Prepare A* data structures
    frontier = PriorityQueue()
    frontier.put(start, 0)
    parent = {start: None}
    g_value = {start: 0}
    # Start an A* search
    while not frontier.empty():
        current_v = frontier.get()
        # If goal reached, return path length
        if current_v == target:
            timesteps = 0
            while current_v != start:
                current_v = parent[current_v]
                timesteps += 1
            return timesteps
        # Expand the current node
        for next_v in list(graph.neighbors(current_v))+[current_v]:#graph.adj_list[current_v]:
            new_cost = g_value[current_v] + 1
            if next_v not in g_value or new_cost < g_value[next_v]:
                g_value[next_v] = new_cost
                priority = new_cost + manhattan_distance(next_v, target)
                frontier.put(next_v, priority)
                parent[next_v] = current_v
    # Goal not reached, raise exception
    raise ValueError

def prepare_model(
    graph: nx.Graph,
    starts: List[Tuple[int, int]],
    targets: List[Tuple[int, int]],
    timesteps: int,
    objective: str = "Makespan"
) :
    """Given input problem decription and makespan, generate an ILP model.
    Args:
        graph (Graph): graph
        starts (List[Tuple[int, int]]): a list of start coordinates
        targets (List[Tuple[int, int]]): a list of target coordinates, size must match starts
        timesteps (int): projected makespan
        objective (str, optional): whether to optimize makespan of sum of distance. Defaults to "Makespan".
    Returns:
        gp.Model: an ILP model
        List[Tuple[int, Tuple[int, int], Tuple[int, int], int]]: list of gurobi variables
    """
    
    model = gp.Model('MRMP')
    model.Params.LogToConsole = 0
    num_robots = len(starts)
    objective_expression = gp.LinExpr()
    
    # Data structures for adding constraints
    robot_in_map = dict()  # (r, v, t): vars going into v at t for robot r
    robot_out_map = dict()  # (r, v, t): vars going out from v at t for robot r
    vertex_in_map = dict()  # (v, t): vars going into v at t
    vertex_out_map = dict()  # (v, t): vars going out from v at t
    edge_horizontal = dict()  # (v, t): vars uses horizontal edges to v at t
    edge_vertical = dict()  # (v, t): vars uses vertical edges to v at t
    edge_map = dict(
    )  # (v1, v2, t): vars use the undirectional edge between v1, v2 at t

    # Find all path variables
    variable_keys = list()  # Stores all variable names
    objective_variable_keys = list()
    # print(timesteps,'timesteps')
    for r in range(num_robots):
        reachable_vertices = set()  # Reachable vertices at current step
        reachable_vertices.add(starts[r])
        for t in range(1, timesteps + 1):
            next_reachable_vertices = set()  # Reachable vertices in next step
            for v1 in reachable_vertices:
                for v2 in list(graph.neighbors(v1))+[v1]:
                    # Check vertex reachability: if not reachable, discard
                    if manhattan_distance(targets[r], v2) > timesteps - t:
                        continue
                    next_reachable_vertices.add(v2)
                    name = (
                        r, v1, v2, t - 1
                    )  # Variable name as (robot, from_vertex, to_vertex, time)
                    variable_keys.append(name)
                    if v1!=v2 and objective=="TotalDistance":
                        objective_variable_keys.append(name)
                    # Save variable name for adding constraints
                    dict_insert_helper(robot_out_map, (r, v1, t - 1), name)
                    dict_insert_helper(robot_in_map, (r, v2, t), name)
                    dict_insert_helper(vertex_out_map, (v1, t - 1), name)
                    dict_insert_helper(vertex_in_map, (v2, t), name)
                    if v2 > v1:
                        dict_insert_helper(edge_map, (v1, v2, t), name)
                    elif v1 > v2:
                        dict_insert_helper(edge_map, (v2, v1, t), name)
                    # no cycle!
                    if check_edge_type(name[1], name[2]) == "horizontal":
                        dict_insert_helper(edge_horizontal, (v1, t), name)
                        dict_insert_helper(edge_horizontal, (v2, t), name)
                    elif check_edge_type(name[1], name[2]) == "vertical":
                        dict_insert_helper(edge_vertical, (v1, t), name)
                        dict_insert_helper(edge_vertical, (v2, t), name)
            reachable_vertices = next_reachable_vertices

    # Set model objective
    # TODO we need to add sum of distance objective here
    feed_back_variable_keys=[]
    for r in range(num_robots):
        name = (r, targets[r], starts[r], timesteps)  # Variable name as (robot, from_vertex, to_vertex, time)

        if objective=="Makespan":        
            objective_variable_keys.append(name)
        variable_keys.append(name)
        feed_back_variable_keys.append(name)
        # Save variable name for adding constraints
        dict_insert_helper(robot_out_map, (r, targets[r], timesteps), name)
        dict_insert_helper(robot_in_map, (r, starts[r], 0), name)
        dict_insert_helper(vertex_out_map, (targets[r], timesteps), name)
        dict_insert_helper(vertex_in_map, (starts[r], 0), name)

    grb_variables = model.addVars(variable_keys,vtype=GRB.BINARY)
    objective_expression = gp.LinExpr(
                [1 for name in objective_variable_keys],
                [grb_variables[name] for name in objective_variable_keys])
        # Add all variables to the model
    if objective=="Makespan":
        model.setObjective(objective_expression, GRB.MAXIMIZE)
    elif objective=="TotalDistance":
        model.setObjective(objective_expression,GRB.MINIMIZE)
        constraint_expr = gp.LinExpr([1 for name in feed_back_variable_keys],
                                     [grb_variables[name] for name in feed_back_variable_keys])
        model.addLConstr(constraint_expr,GRB.EQUAL,num_robots)

    


    # Add constraints: robot movement flow constraints
    for key in set(list(robot_in_map.keys()) + list(robot_out_map.keys())):
        try:
            in_variables = [grb_variables[name] for name in robot_in_map[key]]
        except:
            in_variables = []
        try:
            out_variables = [
                grb_variables[name] for name in robot_out_map[key]
            ]
        except:
            out_variables = []
        constraint_expr = gp.LinExpr([1 for i in out_variables] +
                                     [-1 for i in in_variables],
                                     out_variables + in_variables)
        model.addLConstr(constraint_expr, GRB.EQUAL, 0)

    # Add constraints: vector collision constraints
    for key in vertex_in_map.keys():
        in_variables = [grb_variables[name] for name in vertex_in_map[key]]
        constraint_expr = gp.LinExpr([1 for i in in_variables], in_variables)
        model.addLConstr(constraint_expr, GRB.LESS_EQUAL, 1)

    # Add constraints: edge collision constraints
    for key in edge_map.keys():
        variables = [grb_variables[name] for name in edge_map[key]]
        constraint_expr = gp.LinExpr([1 for i in variables], variables)
        model.addLConstr(constraint_expr, GRB.LESS_EQUAL, 1)


    model.update()
    return model, grb_variables


def check_solution_direction(v1: Tuple[int, int], v2: Tuple[int,
                                                            int]):
    """ Check the robot movement direction given pre and after vertex

    Args:
        v1 (Tuple[int, int]): vertex before moving
        v2 (Tuple[int, int]): vertex after moving

    Returns:
        Direction: direction
    """
    if v1 == v2:
        return Direction.WAIT
    elif v1[0] == v2[0] and v1[1] + 1 == v2[1]:
        return Direction.NORTH
    elif v1[0] == v2[0] and v1[1] - 1 == v2[1]:
        return Direction.SOUTH
    elif v1[0] + 1 == v2[0] and v1[1] == v2[1]:
        return Direction.EAST
    elif v1[0] - 1 == v2[0] and v1[1] == v2[1]:
        return Direction.WEST

def check_valid(starts,goals,output):
    for i in range(0,len(starts)):
        if len(output[i])!=0:
            if manhattan_distance(starts[i],output[i][0])>1:
                return False
            if goals[i]!=output[i][-1]:
                return False

    
    
    for path in output:
        for i in range(0,len(path)-1):
            if manhattan_distance(path[i],path[i+1])>1:
                
                return False
    
    for t in range(0,len(output[0])):
        used=[]
        for i in range(0,len(output)):
            if output[i][t] in used:
         
                return False
            used.append(output[i][t])
    return True




def solve(instance: pg.Instance,solution=[]):
    """ Use ILP to solve an instance

    Args:
        instance (Instance): a cg:shop problem instance

    Returns:
        Solution: a solution to the input instance
    """
    #solution = Solution(instance)
    # Construct graph
    graph = instance.graph
    # Get an underestimated makespan
    candidate_timesteps = [
        shortest_path_length(graph, instance.starts[r], instance.goals[r])
        for r in range(instance.number_of_agents)
    ]
    timesteps = max(candidate_timesteps)
    if timesteps==0:
        output=[]
        for start in instance.starts:
            output.append([start])
        return output
    # Solve problem with ILP
    # TODO: need a timeout function

    while True:
        # Construct model
        model, grb_variables = prepare_model(graph, instance.starts,
                                             instance.goals, timesteps)
       
        # Solve
        model.optimize()
        # Check if problem solved
        obj = model.getObjective()

        
        if obj.getValue() == instance.number_of_agents:
            # Retrieve solution
            #steps = [SolutionStep() for t in range(timesteps)]
            output = [[(0, 0) for r in range(timesteps)]
                      for t in range(instance.number_of_agents)
                      ]  # Output solution for validation
            # For each robot, find the correspoinding positive variables
  
            for r in range(instance.number_of_agents):
                current_v = instance.starts[r]
                for t in range(timesteps):
                    found_path=False
                    for neighbor_v in list(graph.neighbors(current_v))+[current_v]:
                        name = (r, current_v, neighbor_v, t)
                        try:
                            if abs(grb_variables[name].x-1) <=1e-3:
                                current_v = neighbor_v
                                output[r][t] = neighbor_v
                                found_path=True
                                break
                        except KeyError:
                            continue
            
                    assert(found_path)
                #assert(output[r][-1]==instance.goals[r])
                # assert(output[r][0]==instance.starts[r])
                solution.append(output[r])
   
            # for t, step in enumerate(output):
            #     print(t, step)

            break
        else:
            timesteps += 1
    # if not check_valid(output):
    #     print(instance.starts)
    #     print(instance.goals)
    #     print(output)
    assert(check_valid(instance.starts,instance.goals,output))
    return output


def test_8():
    graph=nx.grid_graph(dim=[3,3])
    starts=[(0, 0), (1, 2)]
    goals=[(0,1), (2, 1)]
    graph.remove_node((1,1))
    print(list(graph.nodes))
    instance=pg.Instance(starts,goals,graph)
    output=solve(instance)
    print(output)

if __name__ == "__main__":
    #starts,goals=pg.read_instance('./data/problems/instances/random-5x5-obs_pct-0.0/random-5x5-obs_pct-0.0-robot-2-0.instance')
    #graph=pg.read_graph('./data/problems/maps/random-5x5-obs_pct-0.0.map')
    #graph=pg.read_graph('./data/problems/maps/DAO/brc202d.map')
    test_8()
    
    