
import matplotlib.pyplot as plt
import numpy as np
import re
from statistics import mean
from ast import literal_eval as make_tuple



plt.rcParams['text.usetex'] = True
plt.rcParams['font.family'] = "serif"
plt.rcParams['font.serif'] = "Times"
plt.rcParams.update(
        {
            'xtick.labelsize': 22,
            'ytick.labelsize': 22,
            "legend.borderpad": 0.2,  ## border whitespace
            "legend.labelspacing": 0.2,  ## the vertical space between the legend entries
            "legend.handlelength": 1,  ## the length of the legend lines
            "legend.handleheight": 0.7,  ## the height of the legend handle
            "legend.handletextpad": 0.2,  ## the space between the legend line and legend text
            "legend.borderaxespad": 0.5,  ## the border between the axes and legend edge
            "legend.columnspacing": 1.0,  ## column separation
            "legend.framealpha": 0.1
        }
    )
font1 = {'family' : 'Serif',
'weight' : 'normal',
'size'   : 22,
}

def set_title(option):
    if option=="runtime":
        plt.xlabel('Graph size (m)',fontsize=28)
        plt.ylabel('Computation Time (s)',fontsize=28)
    else:
        plt.xlabel('Graph size (m)',fontsize=28)
        plt.ylabel('Optimality Ratio',fontsize=28)
    plt.tick_params(labelsize=22)


def read_result(file_name):
    makespan=0
    lb_makespan=0
    soc=0
    lb_soc=0
    time=0
    num_agents=0
    with open(file_name, "r") as file_content:
        lines = file_content.readlines()
        for line in lines:
            substr=line.split('=')
            string=substr[0]
            if string=='agents':
                num_agents=int(substr[1])
                continue
            if string=='soc':
                soc=int(substr[1])
                continue
            if string=='lb_soc':
                lb_soc=int(substr[1])
                continue
            if string=='makespan':
                makespan=int(substr[1])
                continue
            if string=='lb_makespan':
                lb_makespan=int(substr[1])
                continue
            if string=='comp_time':
                time=float(substr[1])/1000.
                break
        return num_agents,makespan,lb_makespan,soc,lb_soc,time

def test():
    num_agents,makespan,lb_makespan,soc,lb_soc,time=read_result('./ddm/30x30_0.txt')
    print(num_agents,makespan,lb_makespan,soc,lb_soc,time)
       

                
def plot_data():
    ############################################
    size_ecbs3d=[6,9,15]
    mk_data_ecbs3d=[]
    soc_data_ecbs3d=[]
    time_data_ecbs3d=[]
    for m in size_ecbs3d:
        mk_opt=[]
        soc_opt=[]
        time_opt=[]
        for k in range(20):
            file_name='./ecbs3d/'+str(m)+'x'+str(m)+'_'+str(k)+'.txt'
            num_agents,makespan,lb_makespan,soc,lb_soc,time=read_result(file_name)
            mk_opt.append(makespan/lb_makespan)
            soc_opt.append(soc/lb_soc)
            time_opt.append(time)
        mk_data_ecbs3d.append(mean(mk_opt))
        soc_data_ecbs3d.append(mean(soc_opt))
        time_data_ecbs3d.append(mean(time_opt))
        
    #####################################################
    size_rth3d=[6,9,15,30,45,60,75,90,120,150,180]
    mk_data_rth3d=[]
    soc_data_rth3d=[]
    time_data_rth3d=[]
    for m in size_rth3d:
        mk_opt=[]
        soc_opt=[]
        time_opt=[]
        for k in range(20):
            try:
                file_name='./rth3d/'+str(m)+'x'+str(m)+'_'+str(k)+'.txt'

                num_agents,makespan,lb_makespan,soc,lb_soc,time=read_result(file_name)
                mk_opt.append((makespan-1)/lb_makespan)
                soc_opt.append(soc/lb_soc)
                time_opt.append(time)
            except:
                pass
        mk_data_rth3d.append(mean(mk_opt))
        soc_data_rth3d.append(mean(soc_opt))
        time_data_rth3d.append(mean(time_opt))
 
    ##########################################################
    size_rth3d_lba=[6,9,15,30,45,60,75,90,120,150,180]
    mk_data_rth3d_lba=[]
    soc_data_rth3d_lba=[]
    time_data_rth3d_lba=[]
    for m in size_rth3d_lba:
        mk_opt=[]
        soc_opt=[]
        time_opt=[]
        for k in range(20):
            try:
                file_name='./rth3dlba/'+str(m)+'x'+str(m)+'_'+str(k)+'.txt'

                num_agents,makespan,lb_makespan,soc,lb_soc,time=read_result(file_name)
                mk_opt.append((makespan-1)/lb_makespan)
                soc_opt.append(soc/lb_soc)
                time_opt.append(time)
            except:
                pass
        mk_data_rth3d_lba.append(mean(mk_opt))
        soc_data_rth3d_lba.append(mean(soc_opt))
        time_data_rth3d_lba.append(mean(time_opt))
    
    
   

    plt.figure( figsize=(6, 4))
  
    l1,=plt.plot(size_ecbs3d,mk_data_ecbs3d,marker='s')
    l2,=plt.plot(size_rth3d,mk_data_rth3d,marker='v')
    l3,=plt.plot(size_rth3d_lba,mk_data_rth3d_lba,marker='o')
    print(mk_data_rth3d_lba)
    
    plt.legend(handles=[l1,l2,l3],labels=['ECBS','RTH','RTH-LBA'],prop=font1)
    set_title("makespan")
    plt.savefig("makespan.png", bbox_inches="tight", pad_inches=0.05)
    plt.show()
    
    plt.figure( figsize=(6, 4))
    l1,=plt.plot(size_ecbs3d,time_data_ecbs3d,marker='s')
    l2,=plt.plot(size_rth3d,time_data_rth3d,marker='v')
    l3,=plt.plot(size_rth3d_lba,time_data_rth3d_lba,marker='o',linestyle='--')
    plt.yscale('log')
    plt.legend(handles=[l1,l2,l3],labels=['ECBS','RTH','RTH-LBA'],prop=font1)
    set_title("runtime")
    plt.savefig("runtime.png", bbox_inches="tight", pad_inches=0.05)
    plt.show()    
            
plot_data()           
            
    
