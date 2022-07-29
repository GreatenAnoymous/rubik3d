
from typing import List, Tuple
import numpy as np
import json
import  os
import itertools
import errno


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
            
def read_instance(scene_name):
    with open(scene_name, "r") as file_content:
        lines = file_content.readlines()
        starts = list()
        goals = list()
        for line in lines[9:]:
            x1, y1,x2, y2= line.split(',')
            starts.append((int(x1), int(y1),int(z1)))
            goals.append((int(x2), int(y2),int(z2)))
        return starts, goals
        
s1,g1=read_instance("./instances/quasi_obs_rec/6_rec_0.scen")
s2,g2=read_instance("./instances/quasi_obs_rec/6_rec_1.scen")
save_instance_as_txt("./demo_obs.scen",s1,g2,(12,24,6))
