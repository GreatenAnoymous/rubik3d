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


def call_rth3dlba(scen_name,output_name):
    cmd=['./rth3dlba',scen_name,output_name]
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
    ss=[3,9,15,21,30,45,60,75]
    for s in ss:
        for k in range(20):
            scen_name="./instances/quasi_random/"+str(s)+'x'+str(s)+'_'+str(k)+'.scen'
            out_name='./data/random/rth3dlba/'+str(s)+'x'+str(s)+'_'+str(k)+'.txt'
            print(scen_name)
            try:
                call_rth3dlba(scen_name,out_name)  
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
                

def run_random_flat_rth3d():
    ss=[6,9,15,30,45,60,75,90,120,150,180,210,240,270]
    for s in ss:
        for k in range(20):
            scen_name="./instances/quasi_random_flatK6/"+str(s)+'x'+str(s)+'_'+str(k)+'.scen'
            out_name='./data/random_flatK6/rth3dlba/'+str(s)+'x'+str(s)+'_'+str(k)+'.txt'
            print(scen_name)
            try:
                call_rth3dlba(scen_name,out_name)  
            except:
                pass
                
def run_random_flat_ecbs3d():
    ss=[6,9,15,30]
    for s in ss:
        for k in range(20):
            scen_name="./instances/quasi_random_flatK6/"+str(s)+'x'+str(s)+'_'+str(k)+'.scen'
            out_name='./data/quasi_random_flatK6/ecbs3d/'+str(s)+'x'+str(s)+'_'+str(k)+'.txt'
            print(scen_name)
            try:
                call_ecbs(scen_name,out_name) 
            except:
                pass
                
def run_random_rec_rthlba():
    ss=[3,6,9,12,15,18,21,24,27,30,33,36]
    #ss=[24,27,30,33,36]
    for m in ss:
        for k in range(20):
            scen_name="./instances/quasi_random_rec/"+str(m)+'_rec_'+str(k)+'.scen'
            out_name='./data/random_rec/rth3dlba/'+str(m)+'_rec_'+str(k)+'.txt'
            print(scen_name)
            try:
                call_rth3dlba(scen_name,out_name)
            except:
                pass

def run_random_rec_rth():
    ss=[3,6,9,12,15,18,21,24,27,30,33,36]
    #ss=[24,27,30,33,36]
    for m in ss:
        for k in range(20):
            scen_name="./instances/quasi_random_rec/"+str(m)+'_rec_'+str(k)+'.scen'
            out_name='./data/random_rec/rth3d/'+str(m)+'_rec_'+str(k)+'.txt'
            print(scen_name)
            try:
                call_rth3d(scen_name,out_name)
            except:
                
                pass                
            
def run_random_rec_ecbs():
    ss=[3,6,9]
    #ss=[3,6,9,12,15,18,21]
    for m in ss:
        for k in range(20):
            scen_name="./instances/quasi_random_rec/"+str(m)+'_rec_'+str(k)+'.scen'
            out_name='./data/random_rec/ecbs3d/'+str(m)+'_rec_'+str(k)+'.txt'
            print(scen_name)
            try:
                call_ecbs(scen_name,out_name)
            except:
                pass
                
def run_density_rth():
    num_agents=[4800,6000,7200,8400,9600,10800,12000,13200,14400]
    for num in num_agents:
        for k in range(20):
            scen_name="./instances/density/"+str(num)+"_"+str(k)+'.scen'
            out_name='./data/density/rth3d/'+str(num)+"_"+str(k)+'.txt'
            print(scen_name)
            try:
                cmd=['./rthdensity',scen_name,out_name]
                output=check_output(cmd,stderr=STDOUT,timeout=300).decode('utf-8')
            except:
                raise error()
                pass

def run_density_rthLBA():
    num_agents=[4800,6000,7200,8400,9600,10800,12000,13200,14400]
    for num in num_agents:
        for k in range(20):
            scen_name="./instances/density/"+str(num)+"_"+str(k)+'.scen'
            out_name='./data/density/rth3dlba/'+str(num)+"_"+str(k)+'.txt'
            print(scen_name)
            try:
                cmd=['./rthdensityLBA',scen_name,out_name]
                output=check_output(cmd,stderr=STDOUT,timeout=300).decode('utf-8') 
            except:
                pass
                
                
def run_ring_rthlba():
    ss=[3,6,9,12,15,18,21,24,27,30,33,36]
    for m in ss:
       
        scen_name="./instances/rings/"+str(m)+'.scen'
        out_name='./data/rings/rth3dlba/'+str(m)+'.txt'
        print(scen_name)
        try:
            call_rth3dlba(scen_name,out_name)
        except:
            pass
            
def run_ring_rth():
    ss=[3,6,9,12,15,18,21,24,27,30,33,36]
    for m in ss:
       
        scen_name="./instances/rings/"+str(m)+'.scen'
        out_name='./data/rings/rth3d/'+str(m)+'.txt'
        print(scen_name)
        try:
            call_rth3d(scen_name,out_name)
        except:
            pass

def run_blocks_rth():
    ss=[3,6,9,12,15,18,21,24,27,30,33,36]
    d=3
    for m in ss:
        for k in range(20):
            scen_name="./instances/blocks/"+str(m)+'_'+str(k)+'.scen'
            print(scen_name)
            out_name='./data/blocks/rth3d/'+str(m)+' '+str(k)+'.txt'
            try:
                call_rth3d(scen_name,out_name)
            except:
                pass
            
def run_blocks_rthlba():
    ss=[3,6,9,12,15,18,21,24,27,30,33,36]
    d=3
    for m in ss:
        for k in range(20):
            
            scen_name="./instances/blocks/"+str(m)+'_'+str(k)+'.scen'
            print(scen_name)
            out_name='./data/blocks/rth3dlba/'+str(m)+'_'+str(k)+'.txt'
            try:
                call_rth3dlba(scen_name,out_name)
            except:
                pass  

def run_obs_rth():
    ss=[3,6,9,12,15,18,21,24,27,30,33,36]
    for m in ss:
        for k in range(20):
            scen_name="./instances/quasi_obs_rec/"+str(m)+'_rec_'+str(k)+'.scen'
            print(scen_name)
            out_name='./data/obs_rec/rth3d/'+str(m)+'_rec_'+str(k)+'.txt'
            try:
                cmd=['./rth3d_obs',scen_name,out_name]
                output=check_output(cmd,stderr=STDOUT,timeout=300).decode('utf-8')
            except:
                pass

def run_obs_rthlba():
    ss=[3,6,9,12,15,18,21,24,27,30,33,36]
    for m in ss:
        for k in range(20):
            scen_name="./instances/quasi_obs_rec/"+str(m)+'_rec_'+str(k)+'.scen'
            print(scen_name)
            out_name='./data/obs_rec/rth3dlba/'+str(m)+'_rec_'+str(k)+'.txt'
            #try:
            cmd=['./rth3d_obs_lba',scen_name,out_name]
            output=check_output(cmd,stderr=STDOUT,timeout=300).decode('utf-8')
            #except:
                #pass

def run_obs_ecbs():
    ss=[3,6,9]
    for m in ss:
        for k in range(20):
            scen_name="./instances/quasi_obs_rec/"+str(m)+'_rec_'+str(k)+'.scen'
            print(scen_name)
            out_name='./data/obs_rec/ecbs3d/'+str(m)+'_rec_'+str(k)+'.txt'
            
            cmd=['./ecbs3d_obs','-s',scen_name,'-o',out_name,'-w','1.5']
            output=check_output(cmd,stderr=STDOUT,timeout=300).decode('utf-8')
            
def run_ilp_random_rec():
    ss=[3,6,9]
    #ss=[3,6,9,12,15,18,21]
    for m in ss:
        for k in range(20):
            scen_name="./instances/quasi_random_rec/"+str(m)+'_rec_'+str(k)+'.scen'
            out_name='./data/random_rec/ilp16/'+str(m)+'_rec_'+str(k)+'.txt'
            print(scen_name)
            try:
                cmd=['./ilp16',scen_name,out_name]
                output=check_output(cmd,stderr=STDOUT,timeout=300).decode('utf-8')
            except:
                pass
                
def run_obs_ilp():
    ss=[3,6,9]
    for m in ss:
        for k in range(20):
            scen_name="./instances/quasi_obs_rec/"+str(m)+'_rec_'+str(k)+'.scen'
            print(scen_name)
            try:
                out_name='./data/obs_rec/ilp16/'+str(m)+'_rec_'+str(k)+'.txt'
                
                cmd=['./ilp16',scen_name,out_name]
                output=check_output(cmd,stderr=STDOUT,timeout=300).decode('utf-8')
            except:
                pass
            
def run_random_flat_ilp():
    ss=[6,9,15,30]
    for s in ss:
        for k in range(20):
            scen_name="./instances/quasi_random_flatK6/"+str(s)+'x'+str(s)+'_'+str(k)+'.scen'
            out_name='./data/random_flatK6/ilp16/'+str(s)+'x'+str(s)+'_'+str(k)+'.txt'
            print(scen_name)
            try:
                cmd=['./ilp16',scen_name,out_name]
                output=check_output(cmd,stderr=STDOUT,timeout=300).decode('utf-8')
            except:
                pass
            
#run_random_flat_rth3d()
#run_random3d()
#run_random_flat_ecbs3d()
#run_already2()
#run_density_rth()
#run_density_rthLBA()
#run_random_rec_rth()
#run_random_rec_rthlba()
#run_random_rec_ecbs()
#run_ring_rthlba()
#run_ring_rth()
#run_blocks_rth()
#run_blocks_rthlba()

#run_obs_rthlba()
#run_obs_ecbs()
#run_obs_rth()
#run_obs_ilp()
run_ilp_random_rec()
#run_random_flat_ilp()
