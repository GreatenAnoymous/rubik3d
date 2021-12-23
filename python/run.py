import yaml
import os
import argparse
from subprocess import STDOUT, check_output
import problem_generator as pg
import json
import errno
import problem_generator as pg
import networkx as nx
import numpy as np



def distance(s,g):
    return abs(s[0]-g[0])+abs(s[1]-g[1])+abs(s[2]-g[2])




def eval_lb(starts,goals):
    num=len(starts)
    path_length=[distance(starts[i],goals[i]) for i in range(num)]
    makespan_lb=max(path_length)
    soc_lb=sum(path_length)
    return makespan_lb,soc_lb

def read_txt(file_name: str) :
    with open(file_name, "r") as file_content:
        lines = file_content.readlines()
        starts = list()
        goals = list()
        for line in lines[6:]:
            #print(line)
            x1, y1,z1, x2, y2,z1 = line.split(',')
            starts.append((int(x1), int(y1),int(z1)))
            goals.append((int(x2), int(y2),int(z2)))
        return starts, goals    
        
def shrink(paths):
    for p in paths:
        try:
            while p[-1]==p[-2]:
                p.pop(-1)
        except:
            pass

   
def call_ecbs(scen_name,output_name):
    cmd=['./ecbs3d','-s',scen_name,'-o',output_name,'-w','1.5']
    output=check_output(cmd,stderr=STDOUT,timeout=1200).decode('utf-8')
    
   
def call_rth3d(scen_name,output_name):
    cmd=['./rth3d',scen_name,output_name]
    output=check_output(cmd,stderr=STDOUT,timeout=300).decode('utf-8')


def run_already():
    ss=[15]
    for s in ss:
        for k in range(20):
            scen_name="./instances/already_center/"+str(s)+'x'+str(s)+'_'+str(k)+'.scen'
            out_name='./data/already/ecbs3d/'+str(s)+'x'+str(s)+'_'+str(k)+'.txt'
            print(scen_name)
            call_ecbs(scen_name,out_name)


def run_already2():
    ss=[3,9,15,21,30,45,60]
    for s in ss:
        for k in range(20):
            scen_name="./instances/already_center/"+str(s)+'x'+str(s)+'_'+str(k)+'.scen'
            out_name='./data/already/rth3d/'+str(s)+'x'+str(s)+'_'+str(k)+'.txt'
            print(scen_name)
            call_rth3d(scen_name,out_name)  
            

def run_random3d():            
    ss=[3,9,15,21,30,45,60]
    for s in ss:
        for k in range(20):
            scen_name="./instances/quasi_random/"+str(s)+'x'+str(s)+'_'+str(k)+'.scen'
            out_name='./data/quasi_random/rth3d/'+str(s)+'x'+str(s)+'_'+str(k)+'.txt'
            print(scen_name)
            try:
                call_rth3d(scen_name,out_name)  
            except:
                pass

def run_random_ecbs3d():
    ss=[3,9,15]
    for s in ss:
        for k in range(20):
            scen_name="./instances/quasi_random/"+str(s)+'x'+str(s)+'_'+str(k)+'.scen'
            out_name='./data/quasi_random/ecbs3d/'+str(s)+'x'+str(s)+'_'+str(k)+'.txt'
            print(scen_name)
            try:
                call_ecbs(scen_name,out_name)
            except:
                pass
                

            
run_random_ecbs3d()
#run_already2()
