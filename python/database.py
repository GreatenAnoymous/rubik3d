import json
from tracemalloc import start

import problem_generator as pg
from itertools import permutations 
from itertools import combinations
import networkx as nx
from LBA import *
from subprocess import STDOUT, check_output

graph=pg.generate_full_graph(2,3)

# encoding of  figure 8 vertex
def get_id(vertex):
    return vertex[0]+vertex[1]*3

def get_vertex(id):
    vertex_id_map={0:(0,0),1:(1,0),2:(2,0),3:(0,1),4:(1,1),5:(2,1)}
    return vertex_id_map[id]
# encoding of



def manhattan_distance(v1,v2):
    return abs(v1[0]-v2[0])+abs(v1[1]-v2[1])

def solve_instance(start_id,goal_id):
    starts=[get_vertex(id) for id in start_id]
    goals=[get_vertex(id) for id in goal_id]
    #print(starts,goals)
    instance=pg.Instance(starts,goals,graph)
    output=ilp.solve(instance)
    return output


def solve_instance_uniform_obs(start_id,goal_id):
    vertex_id_map={0:(0,0),1:(1,0),2:(2,0),3:(0,1),4:(1,1),5:(2,1),6:(0,2),7:(1,2),8:(2,2)}
    graph=pg.generate_graph(3,3,[(1,1)])
    starts=[vertex_id_map[id] for id in start_id]
    goals=[vertex_id_map[id] for id in goal_id]

    instance=pg.Instance(starts,goals,graph)
    output=ilp.solve(instance)
    return output

def sovle_instance_ilp(start_id,goal_id):
    vertex_id_map={0:(0,0),1:(1,0),2:(2,0),3:(0,1),4:(1,1),5:(2,1),6:(0,2),7:(1,2),8:(2,2)}
    # graph=pg.generate_graph(3,3,[(1,1)])
    graph=pg.generate_graph(3,3,[])
    if len(start_id)==2:        #obstacles
        graph=pg.generate_graph(3,3,[(1,1)])
        print("obstacles!")
    starts=[vertex_id_map[id] for id in start_id]
    goals=[vertex_id_map[id] for id in goal_id]
    pg.write_graph(graph,'./tmp/tmp.map')
    pg.write_instance(graph,starts,goals,'./tmp/tmp.map','./tmp/tmp.instance')
    cmd=['./ecbs','./tmp/tmp.map','./tmp/tmp.instance']
    output=check_output(cmd,stderr=STDOUT,timeout=300).decode('utf-8')
    output_dict=json.loads(output)
    solution=output_dict['solution']
    new_solution=[sol for sol in solution]
    print("solved")
    return new_solution
    

    
def if_neighbor(id1,id2):
    v1=get_vertex(id1)
    v2=get_vertex(id2)
    if manhattan_distance(v1,v2)>1:
        return False
    return True

 

def generate_data():
    #choose for goal:
    data=dict()
    max_makespan=0
    for number_of_agents in range(1,7):
        assert(number_of_agents<=6)
        goal_comb=list(combinations([0,1,2,3,4,5],number_of_agents))
        start_permuatation=list(permutations([0,1,2,3,4,5],number_of_agents))   
        for start_id in start_permuatation:
            for goal_id in goal_comb:
                output=solve_instance(start_id,goal_id)
                key=str((start_id,goal_id))
                paths=[]
                for i in range(0,len(output)):
                    if len(output)!=0:
                        if not if_neighbor(start_id[i],get_id(output[i][0])):
                            print(number_of_agents,key,output)
                        assert(if_neighbor(start_id[i],get_id(output[i][0])))
                for p in output:
                    path=[get_id(v) for v in p]
                    paths.append(path)
                data[key]=paths
                makespan=max(len(path) for path in paths)
                max_makespan=max(makespan,max_makespan)
    exit()
    filename='./fig8_db.json'
    with open(filename,'w') as dump_f:
        json.dump(data,dump_f)
    

def task_assign(start_id,goal_id):
    cost_matrix=[]
    vertex_id_map={0:(0,0),1:(1,0),2:(2,0),3:(0,1),4:(1,1),5:(2,1),6:(0,2),7:(1,2),8:(2,2)}
    for s in start_id:
        cost_i=[]
        vs=vertex_id_map[s]
        for g in goal_id:
            gs=vertex_id_map[g]
            cost_i.append(manhattan_distance(vs,gs))
        cost_matrix.append(cost_i)
    row_ind,col_ind,_=labp_solve(cost_matrix)
    new_goal=[goal_id[col_ind[i]] for i in row_ind]
    return start_id,new_goal

def generate_local3x3_uniform():
    data=dict()

    number_of_agents=1
    goal_comb=[(4,),(4,)]
    start_permutation=list(combinations([0,1,2,3,4,5,6,7,8],number_of_agents))
    
    for start_id in start_permutation:
        for goal_id in goal_comb:
            print(start_id)
            print(goal_id)
            # start_id,goal_id=task_assign(start_id,goal_id)
            # output=solve_instance_uniform_obs(start_id,goal_id)
            output=sovle_instance_ilp(start_id,goal_id)
            paths=[]
            key1=str((start_id,'y'))
           
            key2=str((start_id,'x'))
           
            for p in output:
                path=[get_id(v) for v in p]
                paths.append(path)
            data[key1]=paths
            data[key2]=paths

    number_of_agents=2
    goal_comb=[(1,7),(3,5)]
    start_permutation=list(combinations([0,1,2,3,4,5,6,7,8],number_of_agents))
    for start_id in start_permutation:
        for goal_id in goal_comb:
            start_id,goal_id=task_assign(start_id,goal_id)
            # output=solve_instance_uniform_obs(start_id,goal_id)
            output=sovle_instance_ilp(start_id,goal_id)
            if set(goal_id)==set([1,7]):
                key=str((start_id,'y'))
                print("y")
            else:
                key=str((start_id,'x'))
            paths=[]
            for p in output:
                path=[get_id(v) for v in p]
                paths.append(path)
            data[key]=paths
 
    number_of_agents=3
    goal_comb=[(1,4,7),(3,4,5)]
    start_permutation=list(combinations([0,1,2,3,4,5,6,7,8],number_of_agents))
    for start_id in start_permutation:
        for goal_id in goal_comb:
            start_id,goal_id=task_assign(start_id,goal_id)
         
            # output=solve_instance_uniform_obs(start_id,goal_id)
            output=sovle_instance_ilp(start_id,goal_id)
            if set(goal_id)==set([1,4,7]):
                key=str((start_id,'y'))
                print('y')
            else:
                key=str((start_id,'x'))
            paths=[]
            for p in output:
                path=[get_id(v) for v in p]
                paths.append(path)
            data[key]=paths

    
    filename='./local3x3.json'
    with open(filename,'w') as dump_f:
        json.dump(data,dump_f)


def generate_local3x3_uniform_obs_data():
    data=dict()
    #2 agents
    number_of_agents=2
    
    # goal_comb=list(combinations([0,1,2,3,5,6,7,8],number_of_agents))
    goal_comb=[(1,7),(3,5)]
    start_permutation=list(permutations([0,1,2,3,5,6,7,8],number_of_agents))
    for start_id in start_permutation:
        for goal_id in goal_comb:
            start_id,goal_id=task_assign(start_id,goal_id)
            # print(goal_id)
            # output=solve_instance_uniform_obs(start_id,goal_id)
            output=sovle_instance_cbs(start_id,goal_id)
            if tuple(goal_id)==(1,7) or tuple(goal_id)==(7,1):
                key=str((start_id,'y'))
            else:
                key=str((start_id,'x'))
            paths=[]
            for p in output:
                path=[get_id(v) for v in p]
                paths.append(path)
            data[key]=paths
    filename='./local3x3_uniform_obs_db.json'
    with open(filename,'w') as dump_f:
        json.dump(data,dump_f)

if __name__=='__main__':
    # generate_local3x3_uniform_obs_data()
    generate_local3x3_uniform()

            











