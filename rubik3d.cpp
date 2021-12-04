/**
 * @file rubik3d.cpp
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2021-11-24
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#include "rubik3d.hpp"

////////////////////////////////////////////////////// RTH3D /////////////////////////////////////////////////////////////////////////

RTH_3d::RTH_3d(Robots & _r,Grids3d *_g):robots(_r),graph(_g){

}

void RTH_3d ::solve(){
    prepare();
    //first do fat column (xz-plane) matching 
    lba_matching_fat();
    z_shuffle();
    //for each x-z plane apply 3-m shuffle to get to the matched positions
    // lba_matching_2d();
    xy_fitting();
    xy_shuffle();
    z_fitting();
    z_shuffle();
}

/**
 * @brief 
 * for simplicity assume m=3k
 */
void RTH_3d::prepare(){
    const int cell_size=3;
    // using point3d=std::tuple<int,int,int>;
    robotPlaceMap start_agents;
    robotPlaceMap goal_agents;
    for(auto &r:robots){
        point3d start_id={r->current->x/cell_size,r->current->y/cell_size,r->current->z/cell_size};
        point3d goal_id={r->goal->x/cell_size,r->goal->y/cell_size,r->goal->z/cell_size};
        start_agents[start_id].push_back(r);
        goal_agents[goal_id].push_back(r);
    }

    for(int i=0;i<graph->xmax;i+=cell_size){
        for(int j=0;j<graph->ymax;j+=cell_size){
            for(int k=0;k<graph->zmax;j+=cell_size)
                prepare_helper(i,j,k,start_agents,goal_agents);
        }
    }
}
/**
 * @brief 
 * first matching the fat column
 */
void RTH_3d::lba_matching_fat(){
    using point2d=std::pair<int,int>; //x,y
    std::unordered_map<point2d,Robots,boost::hash<point2d>> fat_column_dict;
    for(int i=0;i<graph->xmax;i++){
        for(int j=1;j<graph->ymax;j+=cell_size){
            fat_column_dict[{i,j}]={};
        }
    }
    for(auto &r:robots){
        point2d key={r->current->x,r->current->y};
        fat_column_dict[key].push_back(r);
    }

    for(int i=0;i<graph->zmax;i++){

    }
    
}



/**
 * @brief 
 * 
 */
void RTH_3d::prepare_helper(int i,int j,int k,robotPlaceMap & start_robots ,robotPlaceMap &goal_robots){

}

void RTH_3d::xy_shuffle(){

    for(int i=0;i<graph->xmax;i+=cell_size){
        
        Robots agents;
        for(auto &r:robots){
            if(r->current->x<i+cell_size and r->current->x>=i) agents.push_back(r);
        }
        Motion3d swapper(agents,{0,graph->xmax-1},{0,i+cell_size-1},{0,0},'x');
        swapper.reconfigure();
    }
    fill_paths(robots);

}


void RTH_3d::z_shuffle(){

}

void RTH_3d::xy_fitting(){
    for(auto &r:robots){
        r->inter_goal=getVertex(0,0,0);
    }
}

void RTH_3d::z_fitting(){
    for(auto &r:robots) r->intermediate=getVertex(0,0,0);
}




