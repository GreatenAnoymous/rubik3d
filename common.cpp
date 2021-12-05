/**
 * @file common.cpp
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2021-12-04
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#include"common.hpp"



/**
 * @brief Construct a new Grids 3d:: Grids 3d object
 * 
 * @param xmax 
 * @param ymax 
 * @param zmax 
 */
Grids3d::Grids3d(int _xmax,int _ymax,int _zmax):xmax(_xmax),ymax(_ymax),zmax(_zmax){
    auto getId=[&](int x,int y,int z){
        return x+xmax*y+xmax*ymax*z;
    };
    nodes=std::vector<Location3d*>(xmax*ymax*zmax,nullptr);
    for(int x=0;x<xmax;x++){
        for(int y=0;y<ymax;y++){
            for(int z=0;z<zmax;z++){
                Location3d* node=new Location3d(getId(x,y,z),x,y,z);
                nodes[getId(x,y,z)]=node;
            }
        }
    }
}

Grids3d::~Grids3d(){
    for(auto &n:nodes) delete n;
}

/**
 * @brief 
 * 
 * @param id 
 * @return Location3d* 
 */

Location3d* Grids3d::getVertex(int id){
    return nodes[id];
}

/**
 * @brief 
 * 
 * @param x 
 * @param y 
 * @param z 
 * @return Location3d* 
 */
Location3d* Grids3d::getVertex(int x,int y, int z){
    // std::cout<<"grid debug "<<nodes.size()<< " "<<x+xmax*y+xmax*ymax*z<<std::endl;
    return nodes[x+xmax*y+xmax*ymax*z];
}

/**
 * @brief 
 * 
 * @param node 
 * @return std::vector<Location3d*> 
 */
std::vector<Location3d*> Grids3d::getNeighbors(Location3d * node){
    std::vector<Location3d *> neighbors;
    if(node->x-1 >=0 ) neighbors.push_back(getVertex(node->x-1,node->y,node->z));
    if(node->y-1>=0) neighbors.push_back(getVertex(node->x,node->y-1,node->z));
    if(node->z-1>=0) neighbors.push_back(getVertex(node->x,node->y,node->z-1));
    if(node->x+1<xmax) neighbors.push_back(getVertex(node->x+1,node->y,node->z));
    if(node->y+1<ymax) neighbors.push_back(getVertex(node->x,node->y+1,node->z));
    if(node->z+1<zmax) neighbors.push_back(getVertex(node->x,node->y,node->z+1));
    return neighbors;
}


///////////////////////////////////////////////////////////////////// util functions //////////////////////////////////////////////////

/**
 * @brief 
 * 
 * @param file_name 
 * @param robots 
 * @param xmax 
 * @param ymax 
 * @param zmax 
 */
void read_instances(std::string file_name,Robots &robots,Grids3d *&graph){
  
    int xmax=-1,ymax=-1,zmax=-1;
    std::string line;
    std::smatch results;
    std::ifstream scen_file(file_name);
    std::regex r_xmax=std::regex(R"(xmax=(\d+))");
    std::regex r_ymax=std::regex(R"(ymax=(\d+))");
    std::regex r_zmax=std::regex(R"(zmax=(\d+))");
    std::regex r_sg=std::regex(R"((\d+),(\d+),(\d+),(\d+),(\d+),(\d+))");


    if(!scen_file){
        std::cout<<"File not found!"<<std::endl;
        exit(0);
    }
    int id=0;
    while(getline(scen_file,line)){
        //CRLF
        if(*(line.end()-1)==0x0d) line.pop_back();
        if(std::regex_match(line,results,r_xmax)){
            xmax=std::stoi(results[1].str());
            continue;
        }

        if(std::regex_match(line,results,r_ymax)){
            ymax=std::stoi(results[1].str());
            continue;
        }

        if(std::regex_match(line,results,r_zmax)){
            zmax=std::stoi(results[1].str());
            graph=new Grids3d(xmax,ymax,zmax);
            continue;
        }
        // if(xmax>0 and ymax >0 and zmax>0){
            
        // }
        
        if(std::regex_match(line,results,r_sg)){
            
            int x_s=std::stoi(results[1].str());
            int y_s=std::stoi(results[2].str());
            int z_s=std::stoi(results[3].str());
            int x_g=std::stoi(results[4].str());
            int y_g=std::stoi(results[5].str());
            int z_g=std::stoi(results[6].str());
            Robot *ri=new Robot(id,graph->getVertex(x_s,y_s,z_s),graph->getVertex(x_g,y_g,z_g));
            id++;
      
            robots.push_back(ri);
            
            // starts.push_back(Location(x_s,y_s));
            // goals.push_back(Location(x_g,y_g));
            continue;
        }
        
    } 
}

void evaluate_result(Robots &robots,int &makespan,int &makespanLB,int &soc,int &socLB){
    makespan=makespanLB=soc=socLB=0;
    for(auto &r:robots){
        makespan=std::max(makespan,(int)r->path.size());
        soc+=r->path.size();
        socLB+=r->start->manhattan_dist(r->goal);
        makespanLB=std::max(makespanLB,r->start->manhattan_dist(r->goal));
        
    }
}

/**
 * @brief 
 * 
 * @param file_name 
 * @param robots 
 * @param runtime 
 * @param save_paths 
 */
void save_solutions(std::string file_name,Robots&robots,double runtime,bool save_paths){
    std::ofstream out(file_name);
    int makespanLB,socLB,makespan,soc;
    evaluate_result(robots,makespan,makespanLB,soc,socLB);
    out<<"soc="<<soc<<std::endl;
    out<<"lb_soc="<<socLB<<std::endl;
    out<<"makespan="<<makespan<<std::endl;
    out<<"lb_makespan="<<makespanLB<<std::endl;
    out<<"comp_time="<<(int)(runtime*1000)<<std::endl;
    if(save_paths==false) return;
    out<<"solutions="<<std::endl;
    for(int i=0;i<robots.size();i++){
        out<<i<<":";
        for(const auto &v:robots[i]->path){
            out<<'('<<v->x<<","<<v->y<<","<<v->z<<"),";
        }
        out<<std::endl;
    }
}

/**
 * @brief 
 * 
 * @param robots 
 * @param makespan 
 */
void fill_paths(Robots &robots,int makespan){
    if(makespan==-1){
        for(auto &r:robots){
            makespan=std::max(makespan,(int)r->path.size());
        }
    }
    for(auto &r:robots){
        while(r->path.size()<makespan) r->path.push_back(r->path.back());
        r->current=r->path.back();
    }
    
}

/**
 * @brief 
 * remove waitings at the goal
 * @param robots 
 */
void shrink_paths(Robots&robots){
    for(auto &r:robots){
        if(r->path.size()<=1) continue;
        while(r->path.back()==r->path.end()[-2]) r->path.pop_back();
    }
}
