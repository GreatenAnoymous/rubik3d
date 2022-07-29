
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
    starts=[(i,j,k) for i in range(xmax) for j in range(ymax) for k in range(zmax)]
    goals=starts.copy()
    np.random.shuffle(starts)
    np.random.shuffle(goals)
    starts=starts[0:num_agents]
    goals=goals[0:num_agents]
    return starts,goals

def generate_quasi_random_instance(graph_size:Tuple,num_agents=-1):
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
                starts.extend(tmp_starts[0:9])
                goals.extend(tmp_goals[0:9])
    np.random.shuffle(starts)
    np.random.shuffle(goals)
    return starts,goals

def generate_quasi_random_obstacles(graph_size,num_agents=-1):
    xmax,ymax,zmax=graph_size
    if num_agents==-1:
        num_agents=int(xmax*ymax*zmax*2/9)
    starts=[]
    goals=[]
    obstacles=set()
    for z in range(0,zmax):
        for x in range(1,xmax,3):
            for y in range(1,ymax,3):
                obstacles.add((x,y,z))
    ########################################
    for i in range(0,xmax,3):
        for j in range(0,ymax,3):
            for k in range(0,zmax,3):
                tmp_starts=[(x,y,z) for x in range(i,i+3) for y in range(j,j+3) for z in range(k,k+3) if (x,y,z) not in obstacles]
                tmp_goals=tmp_starts.copy()
                np.random.shuffle(tmp_starts)
                np.random.shuffle(tmp_goals)
                starts.extend(tmp_starts[0:6])
                goals.extend(tmp_goals[0:6])
    np.random.shuffle(starts)
    np.random.shuffle(goals)
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

def generate_debug():
    xmax,ymax,zmax=(3,3,3)
    z0=3
    starts=[(x,y,z) for x in range(0,xmax) for y in range(0,ymax) for z in range(0,zmax)]
    goals=starts.copy()
    np.random.shuffle(starts)
    np.random.shuffle(goals)
    num=int(xmax*ymax*zmax/3)
    starts=starts[0:num]
    goals=goals[0:num]
    save_instance_as_txt("./debug.scen",starts,goals,(xmax,ymax,zmax))
    
def quasi_random_data():
    ss=[75]
    for m in ss:
        for k in range(20):
            file_name="./instances/quasi_random/"+str(m)+'x'+str(m)+'_'+str(k)+'.scen'
            print(file_name)
            starts,goals=generate_quasi_random_instance((m,m,m))
            save_instance_as_txt(file_name,starts,goals,(m,m,m))
        #print(ok)
        
def generate_3d_debug():
    ss=[3,9,15,21,30,45,60]
    for m in ss:
        for v in range(20):
            xmax,ymax,zmax=(m,m,m)
            starts=[]
            goals=[]
            for k in range(0,zmax):
                for i in range(1,xmax,3):
                    for j in range(0,ymax):
                        starts.append((i,j,k))
                        goals.append((i,j,k))
            np.random.shuffle(starts)
            np.random.shuffle(goals)
            save_instance_as_txt("./instances/already_center/"+str(m)+'x'+str(m)+'_'+str(v)+'.scen',starts,goals,(xmax,ymax,zmax))
            

def generate_flat_quasi_random():
    K=6
    ss=[210,240,270,300]
    for m in ss:
        for k in range(20):
            file_name="./instances/quasi_random_flatK6/"+str(m)+'x'+str(m)+'_'+str(k)+'.scen'
            print(file_name)
            starts,goals=generate_quasi_random_instance((m,m,K))
            save_instance_as_txt(file_name,starts,goals,(m,m,K))
            
def generate_quasi_random_rec():
    #ss=[3,6,9,12,15,18,21]
    ss=[24,27,30,33,36]
    for m in ss:
        m2=4*m
        m1=2*m
        for k in range(20):
            file_name="./instances/quasi_random_rec/"+str(m)+'_rec_'+str(k)+'.scen'
            starts,goals=generate_quasi_random_instance((m1,m2,m))
            save_instance_as_txt(file_name,starts,goals,(m1,m2,m))

def generate_density_rec():
    #using 120x60x6
    num_agents=[4800,6000,7200,8400,9600,10800,12000,13200,14400]
    for m in num_agents:
        m2=120
        m1=60
        m3=6
        for k in range(20):
            file_name="./instances/density/"+str(m)+'_'+str(k)+".scen"
            starts,goals=generate_random_instance((m1,m2,m3),m)
            save_instance_as_txt(file_name,starts,goals,(m1,m2,m3))
            
            
                    
def generate_ring():
    #2:2:1
    ss=[3,6,9,12,15,18,21,24,27,30,33,36]
    for m in ss:
        starts=[]
        goals=[]
        m2=2*m
        for zi in range(m):
            for i in range(1,m2,3):
                low_x=i
                high_x=m2-1-i
                low_y=i
                high_y=m2-1-i
                start_set=set()
                start_set.update([(i,low_y,zi) for i in range(low_x,high_x+1)])
                start_set.update([(i,high_y,zi) for i in range(low_x,high_x+1)])
                start_set.update([(low_x,i,zi) for i in range(low_y,high_y+1)])
                start_set.update([(high_x,i,zi) for i in range(low_y,high_y+1)])
                
                starts.extend(list(start_set))
    
        for start in starts:
                
            goal=(m2-1-start[0],m2-1-start[1],m-start[2]-1)
           # print(goal,start,"@@")
            goals.append(goal)
           
        print(len(goals),len(set(goals)),len(starts),len(set(starts)))
        assert(len(starts)==int(m*m2*m2/3))
        if len(goals)!=len(set(goals)):
            print(len(goals),len(set(goals)))
        assert(len(goals)==len(set(goals)))
        assert(len(starts)==len(set(starts)))
        
        print(len(starts),len(set(starts)),len(set(goals)),m)
        filename='./instances/rings/'+str(m)+'.scen'
        save_instance_as_txt(filename,starts,goals,(m2,m2,m))
    
def generate_blocks():
    ss=[3,6,9,12,15,18,21,24,27,30,33,36]
    d=3
    for m in ss:
        mk=2*m
        for k in range(0,20):
            starts,goals=generate_quasi_random_instance((mk,mk,m))
            mx=int(mk/d)
            mz=int(m/d)
            ids=[]
            start_ids=[(i,j,l) for i in range(0,mx) for j in range(0,mx) for l in range(0,mz)]
            goal_ids=[(i,j,l) for i in range(0,mx) for j in range(0,mx) for l in range(0,mz)]
            np.random.shuffle(goal_ids)
            #find_agents:
            agents_groups=dict()
            for goal in goals:
                goal_id=(int(goal[0]/d),int(goal[1]/d),int(goal[2]/d))
                if goal_id not in agents_groups:
                    agents_groups[goal_id]=[]
                agents_groups[goal_id].append(goal)
            new_goals=[]
            for start in starts:
                start_id=(int(start[0]/d),int(start[1]/d),int(start[2]/d))
                goal_id=goal_ids[start_ids.index(start_id)]
                new_goal=agents_groups[goal_id].pop()
                ids.append(start_ids.index(start_id))
                new_goals.append(new_goal)
            print(len(starts),len(set(starts)),len(set(new_goals)),m)
            filename='./instances/blocks/'+str(m)+'_'+str(k)+'.scen'
          #  agents=[]
      
        #    for i in range(len(starts)):
         #       agent=Agent(i,starts[i],new_goals[i],virtual=False,current=starts[i],group_id=ids[i])
         #       agents.append(agent)
            save_instance_as_txt(filename,starts,new_goals,(mk,mk,m))

def generate_block_demo():
    m=9
    mk=9
    d=3
    starts,goals=generate_quasi_random_instance((mk,mk,m))
    mx=int(mk/d)
    mz=int(m/d)
    ids=[]
    start_ids=[(i,j,l) for i in range(0,mx) for j in range(0,mx) for l in range(0,mz)]
    goal_ids=[(i,j,l) for i in range(0,mx) for j in range(0,mx) for l in range(0,mz)]
    np.random.shuffle(goal_ids)
    #find_agents:
    agents_groups=dict()
    for goal in goals:
        goal_id=(int(goal[0]/d),int(goal[1]/d),int(goal[2]/d))
        if goal_id not in agents_groups:
            agents_groups[goal_id]=[]
        agents_groups[goal_id].append(goal)
    new_goals=[]
    for goal_id,agents in agents_groups.items():
        print(goal_id,len(agents))
  
    for start in starts:
        start_id=(int(start[0]/d),int(start[1]/d),int(start[2]/d))
        #print(start_id)
        goal_id=goal_ids[start_ids.index(start_id)]
        new_goal=agents_groups[goal_id].pop()
       
        ids.append(start_ids.index(start_id))
        new_goals.append(new_goal)
    print(len(starts),len(set(starts)),len(set(new_goals)),m)
    filename='./demo_block.scen'
    save_instance_as_txt(filename,starts,new_goals,(mk,mk,m))
      
def generate_obstacles():
    ss=[3,6,9,12,15,18,21,24,27,30,33,36]
    for m in ss:
        m2=4*m
        m1=2*m
        for k in range(20):
            file_name="./instances/quasi_obs_rec/"+str(m)+'_rec_'+str(k)+'.scen'
            print(file_name)
            starts,goals=generate_quasi_random_obstacles((m1,m2,m))
            save_instance_as_txt(file_name,starts,goals,(m1,m2,m))
            

def generate_true_random_rec():
    ss=[3,6,9,12,15,18,21,24,27,30,33,36]
    for m in ss:
        m2=4*m
        m1=2*m
        for k in range(20):
            file_name="./instances/true_random/"+str(m)+'_rec_'+str(k)+'.scen'
            starts,goals=generate_random_instance((m1,m2,m))
            save_instance_as_txt(file_name,starts,goals,(m1,m2,m))    
    
    
if __name__=="__main__":
    #graph_size=(9,9,9)
    #generate_debug_rth2d()
    #quasi_random_data()
    #generate_3d_debug()
    #generate_flat_quasi_random()
    #generate_quasi_random_rec()
    #generate_density_rec()
    # generate_quasi_random_rec()
    #generate_obstacles()
    #generate_blocks()
    # generate_true_random_rec()
    generate_block_demo()
