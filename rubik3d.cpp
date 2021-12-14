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
#include <random>
#include "umapf3d.hpp"
#include"labp.hpp"
#include "rth2d.hpp"
#include "mp3d.hpp"
////////////////////////////////////////////////////// RTH3D /////////////////////////////////////////////////////////////////////////

RTH_3d::RTH_3d(Robots & _r,Grids3d *_g):robots(_r),graph(_g){

}

void RTH_3d ::solve(){
    prepare();
    //first do fat column (xz-plane) matching 
    lba_matching_fat();
    z_shuffle();
    //for each x-z plane apply 3-m shuffle to get to the matched positions

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
    
    // std::unordered_map<point2d,Robots,boost::hash<point2d>> fat_column_dict;
    std::unordered_map<int,Robots> fat_column_dict;
    for(auto &r:robots){
        // point2d key={r->current->x,r->current->y};
        int key=get_plane_id(r->current->x,r->current->y);
        fat_column_dict[key].push_back(r);
    }

    for(int i=0;i<graph->zmax;i++){
        std::unordered_map<int,int>matching;
        std::unordered_map<point2d,Robot*,boost::hash<point2d>> arranged_robots;
        matching_helper(fat_column_dict,matching,arranged_robots,i);
        for(int xi=1;xi<graph->xmax;xi+=cell_size){
            for(int yi=1;yi<graph->ymax;yi+=cell_size){
                int key=get_plane_id(xi,yi);
                int column=matching[key];
                int xt,yt;
                arranged_robots[{key,column}]->intermediate=
                    getVertex(xt,yt,i);
            }
        }
    }
    // matching_heuristic();
}

/**
 * @brief 
 * 
 * @param column_dict 
 * @param matching 
 * @param arranged_agents 
 * @param row 
 */
void RTH_3d::matching_helper(std::unordered_map<int,Robots> &column_dict,
        std::unordered_map<int,int> &matching,
        std::unordered_map<point2d,Robot*,boost::hash<point2d>> &arranged_agents,
        int row){
    using weighted_edge=std::tuple<int,int,double>;
    std::vector<weighted_edge> costEdge;
    const int max_inf=1e6;
    std::vector<int> column_id;
    for(auto const &column_i:column_dict){
        column_id.push_back(column_i.first);
    }  
    for(auto const &i: column_id){
        for(auto const &j:column_id){
            bool found=false;
            int min_d=max_inf;
            Robot* min_agent;
            for(auto const &agent_i :column_dict[i]){
                if(get_plane_id(agent_i->umapf_goal->y==j, agent_i->umapf_goal->x)==j and abs(agent_i->current->z-row)<min_d){
                    found=true;
                    min_d=abs(agent_i->current->z-row);
                    min_agent=agent_i;
                }
            }
            if(found){
                costEdge.push_back({i,j,min_d});
                arranged_agents[{i,j}]=min_agent;
            }
            else{
                //(cost_matrix.back()).push_back(max_inf);
            }
        }
    }
    // for(auto &e:costEdge){
    //     std::cout<<std::get<0>(e)<<" "<<std::get<1>(e)<<" "<<std::get<2>(e)<<std::endl;
    // }
    // std::cout<<costEdge.size()<<std::endl;
    std::vector<int> assignment;
    double cost=lba_sparse(costEdge,assignment);
    for(auto c:column_id){
        int color=assignment[c];
        Robot* agent=arranged_agents[{c,color}];
        auto it = std::find(column_dict[c].begin(), column_dict[c].end(), agent);
        if(it!=column_dict[c].end()){
            column_dict[c].erase(it);  
        }else{
            throw std::runtime_error("NOT found the agent!");
        }
    }
    for(auto c:column_id){
        matching.insert({c,assignment[c]});
    }                

}

/**
 * @brief if the starts and goals are balanced configurations 
 * use this function    
 * or use the umapf
 */
void RTH_3d::prepare_helper(int i,int j,int k,robotPlaceMap & start_robots ,robotPlaceMap &goal_robots){

}

void RTH_3d::xy_shuffle(){
    for(int i=0;i<graph->zmax;i++){
        Robots agents;
        for(auto &r:robots){
            if(r->current->z==i) agents.push_back(r);
        }
        RTH_2d swapper(agents,graph);   // for fat column we use RTH2D to simulate the shuffles
  
        swapper.solve();
    }
    fill_paths(robots);

}


void RTH_3d::z_shuffle(){
    for(int i=0;i<graph->xmax;i+=cell_size){
        for(int j=0;j<graph->ymax;j++){
            Robots agents;
            for(auto &r:robots){
                if(r->current->x<i+cell_size and r->current->x>=i and r->current->y==j) agents.push_back(r);
            }
            Motion3d swapper(agents,{i,i+cell_size-1},{j,j},{0,graph->zmax-1},'z');
            swapper.reconfigure();
        }
    }
    fill_paths(robots);
}

void RTH_3d::xy_fitting(){
    for(auto &r:robots){
        r->inter_goal=getVertex(r->umapf_goal->x,r->umapf_goal->y,r->current->z);
    }
}

void RTH_3d::z_fitting(){
    for(auto &r:robots) r->intermediate=getVertex(r->current->x,r->current->y,r->umapf_goal->z);
}



void RTH_3d::random_to_balanced(){
    Configs bl_config;
    for(int i=0;i<graph->xmax;i+=3){
        for(int j=0;j<graph->ymax;j+=3){
            for(int k=0;k<graph->zmax;k+=3){
                Configs tmp_list={getVertex(i+1,j,k),getVertex(i+1,j+1,k),getVertex(i+1,j+2,k),
                    getVertex(i+1,j,k+1),getVertex(i+1,j+1,k+1),getVertex(i+1,j+2,k+1),
                    getVertex(i+1,j,k+2),getVertex(i+1,j+1,k+2),getVertex(i+1,j+2,k+2)};
                bl_config.insert(bl_config.end(),tmp_list.begin(),tmp_list.end());
            }     
        }
    }
    auto rng=std::default_random_engine {};
    std::shuffle(bl_config.begin(),bl_config.end(),rng);
    Configs starts,goals;
    for(auto agent :robots){
        starts.push_back(agent->start);
        goals.push_back(agent->goal);
    }

    //maybe we can use multi-threads to boost the performance
    UMAPF3d solver1(starts,bl_config,graph);
    UMAPF3d solver2(goals,bl_config,graph);
    solver1.solve();
    solver2.solve();
    Paths3d paths_s=solver1.get_result();// start to balanced
    Paths3d paths_g=solver2.get_result();   //goal to balanced

    for(int i=0;i<robots.size();i++){
        // agents[i]->start=paths_s[i].back();
        // agents[i]->goal=paths_g[i].back();
        robots[i]->current=paths_s[i].back();
        robots[i]->inter_goal=paths_g[i].back();
        robots[i]->path.insert(robots[i]->path.end(),paths_s[i].begin()+1,paths_s[i].end());
        robots[i]->umapf_goal_path=paths_g[i];
        std::reverse(robots[i]->umapf_goal_path.begin(),robots[i]->umapf_goal_path.end());
    }
}


