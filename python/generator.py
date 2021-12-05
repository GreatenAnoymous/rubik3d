from typing import List, Tuple
import numpy as np
import json
import  os
import itertools
import errno

#for drones, we assume there is no obstacle

def generate_random_instance(graph_size:Tuple,num_agents=-1):
    xmax,ymax,zmax=graph_size
    if num_agents==-1:
        num_agents=int(xmax*ymax*zmax/3)
    starts=[]
    goals=[]
    return 

def generate_psuedo_random_instance(graph_size:Tuple,num_agents=-1):
    xmax,ymax,zmax=graph_size
    if num_agents==-1:
        num_agents=int(xmax*ymax*zmax/3)
    starts=[]
    goals=[]
    for i in range(0,xmax,3):
        for j in range(0,ymax,3):
            for k in range(0,zmax,3):
                tmp_starts=[(x,y,z) for x in range(i,i+3) for y in range(j,j+3) for z in range(k,k+3)]
                tmp_goals=tmp_starts.copy()
                np.random.shuffle(tmp_starts)
                np.random.shuffle(tmp_goals)
                starts.extend(tmp_starts[0:3])
                goals.extend(tmp_goals[0:3])
    return starts,goals

def generate_test_rth2d(graph_size:Tuple,num_agents=-1):
    #fix z
    z=0   
    xmax,ymax,zmax=graph_size
    if num_agents==-1:
        num_agents=int(xmax*ymax/3)
    starts=[]
    goals=[]
    for i in range(0,xmax,3):
        for j in range(0,ymax,3):
            tmp_start=[(x,y,z) for x in range(i,i+3) for y in range(i,i+3)]
            tmp_goal=tmp_start.copy()
            np.random.shuffle(tmp_start)
            np.random.shuffle(tmp_goal)
            starts.extend(tmp_start[0:3])
            goals.extend(tmp_goal[0:3])
    return starts,goals

def save_instance_as_txt(scene_name:str,starts,goals,graph_size):
    xmax,ymax,zmax=graph_size
    if not os.path.exists(os.path.dirname(scene_name)):
        try:
            os.makedirs(os.path.dirname(scene_name))
        except OSError as exc: # Guard against race condition
            if exc.errno != errno.EEXIST:
                raise
    with open(scene_name, "w") as file_content:
        map_n=str(xmax)+'x'+str(ymax)+'x'+str(zmax)+".map"
        file_content.write("map_file="+map_n+"\n")
        file_content.write("xmax="+str(xmax)+"\n")
        file_content.write("ymax="+str(ymax)+"\n")
        file_content.write("zmax="+str(zmax)+"\n")
        file_content.write("agents="+str(len(starts))+"\n")
        file_content.write("seed=0\n")
        file_content.write("random_problem=0\n")
        file_content.write("max_timestep=3000\n")
        file_content.write("max_comp_time=120000\n")
        for s,g in zip(starts,goals):
            file_content.write(str(s[0])+","+str(s[1])+","+str(s[2])+","+str(g[0])+","+str(g[1])+","+str(g[2])+"\n")
        
    
if __name__=="__main__":
    graph_size=(9,9,9)
    starts,goals=generate_test_rth2d(graph_size)
    save_instance_as_txt("./test.txt",starts,goals,graph_size)