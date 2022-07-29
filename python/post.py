from operator import truediv
import numpy as np
import os
from ast import literal_eval as make_tuple
import collections

def load_schedule(file_name):
    soc=0
    makespan=0
    paths=[]
    with open(file_name, "r") as file_content:
        lines = file_content.readlines()
        #print(lines)
        for line in lines:
            substrx=line.split('=')
            try:
                if substrx[0]=='soc':
                    soc=int(substrx[1])
                if substrx[0]=='makespan':
                    makespan=int(substrx[1])       
            except:
                pass
            try:
                substr_sol=line.split(':')
                
                if len(substr_sol)>=2:
                    #print(substr_sol)
                    path=[]
                    path_string=substr_sol[1]
                    
                    vertex_strings=path_string.split('),')
             
                    for vs in vertex_strings:
                        if vs!='\n':
                            vertex=vs+')'
                            path.append(make_tuple(vertex))
                    
                paths.append(path)
            except:
                pass

    return paths


def shrink_paths(paths):
    t=1
    while t<len(paths[0]):
        rep=True
        for i in range(len(paths)):
            if paths[i][t]!=paths[i][t-1]:
                rep=False
                break
        if rep==True:
            for i in range(len(paths)):
                paths[i]=paths[i][0:t]+paths[i][t+1:]
        else:
            t=t+1
    #print(paths)

def save_paths(paths,file_name,dims=None):
    with open(file_name, "w") as file_content:
        if dims is not None:
            xmax,ymax,zmax=dims
            file_content.write("xmax="+str(xmax)+'\n')
            file_content.write("ymax="+str(ymax)+'\n')
            file_content.write("zmax="+str(zmax)+'\n')

        file_content.write("solution=\n")
        for i,p in enumerate(paths):
            file_content.write(str(i)+':')
            for v in p:
                file_content.write('('+str(v[0])+','+str(v[1])+','+str(v[2])+'),')
            file_content.write('\n')

def format(paths):
    makespan=max(len(p) for p in paths)
    for p in paths:
        while len(p)<makespan:
            p.append(p[-1])
    
def shrink(paths):
    for p in paths:
        if len(p)<2:
            continue
        if p[-1]==p[-2]:
            p.pop()

def remove_wait(paths):
    for p in paths:
        i=0
        while i<len(p):
            if i==len(p)-1:
                i=i+1
                continue
            while (i+1)<len(p) and p[i+1]==p[i]:
                p.pop(i)
            i=i+1

def check_collisions(paths):
    num_agents=len(paths)
    format(paths)
    makespan=len(paths[0])
    for t in range(1,makespan):
        for i in range(num_agents):
            for j in range(i+1,num_agents):
                if paths[i][t]==paths[j][t]:
                    print("find vertex collisions at timestep ",t,paths[i][t],i,j)
                    return
                if paths[i][t-1]==paths[j][t] and paths[j][t-1]==paths[i][t]:
                    print("edge collisions at time step ",t,i,j)
                    return            
# paths=load_schedule("../ru_path.txt")
paths=load_schedule("./test.txt")
#remove_wait(paths)
# path_set=[list(collections.OrderedDict.fromkeys(p)) for p in paths]
#shrink(paths)
shrink_paths(paths)
#print(len(paths[0]))
#shrink_paths(paths)
print(len(paths[0]))
#save_paths(paths,"./ru_path2.txt",(6,9,3))
#check_collisions(paths)
#makespan=max([len(p) for p in paths])
#print(makespan)
save_paths(paths,"./demo_large_obs.txt",(18,36,9))
