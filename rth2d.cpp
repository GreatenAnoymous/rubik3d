/**
 * @file rth2d.cpp
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2021-12-04
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#include"rth2d.hpp"
#include"labp.hpp"

/////debug function////////////////////////////////////////////////




///////////////////////////////RTH2D fat column //////////////////

RTH_2d::RTH_2d(Robots &_r,Grids3d *_graph):robots(_r),graph(_graph){
    assert(robots.size()<=graph->xmax*graph->ymax/3);
}

Location3d* RTH_2d::getV(int x,int y,int z){
    return graph->getVertex(x,y,z);
}

void RTH_2d::solve(){
    prepare();
    matching();
    x_shuffle();
     
    y_fitting();
    y_shuffle();

    x_fitting();
    x_shuffle();
         
    // for(auto &r:robots) assert(r->current==r->inter_goal);
  
    // std::cout<<"solved!"<<std::endl;
}

//move the robots to the middle line if needed
//so here we may assume it is already centered
void RTH_2d::prepare(){

}

//lba matching 
void RTH_2d::matching(){
    std::unordered_map<int,Robots> column_dict;
    int zs;
    for(auto &r:robots){

        int ci=r->current->y;
        column_dict[ci].push_back(r);
        zs=r->current->z;
    }


    for(int i=0;i<graph->xmax/3;i++){
        // std::cout<<"the "<<i<<"-th matching"<<std::endl;
        std::unordered_map<int,int>matching;
        std::unordered_map<point2d,Robot*,boost::hash<point2d>> arranged_robots;
        matching_helper(column_dict,matching,arranged_robots,3*i+1);
        for(int color=0;color<graph->ymax;color++){
            int column=matching[color];
            arranged_robots[{color,column}]->intermediate=
                getV(3*i+1,color,zs);
        }
    }
    // LBA_heuristic();
}

/**
 * @brief 
 * find the minimum bottleneck cost matching
 * @param column_dict 
 * @param matching 
 * @param arranged_agents 
 * @param row 
 */
void RTH_2d::matching_helper(std::unordered_map<int,Robots> &column_dict,
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
                if(agent_i->inter_goal->y==j and abs(agent_i->current->x-row)<min_d){
                    found=true;
                    min_d=abs(agent_i->current->x-row);
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


void RTH_2d::LBA_heuristic(){
 
    std::unordered_map<int,Robots> row_dict;
   
    for(auto &agent:robots){
        row_dict[agent->intermediate->x].push_back(agent);
    }

    std::vector<std::vector<double>>cost_matrix;
    std::vector<int> rows;
    for(const auto &pair:row_dict){
        rows.push_back(pair.first);
    }
   
    for(const auto &row:rows){
        // Agents as=row_dict[row];
        std::vector<double> cost_i;
        cost_matrix.push_back(cost_i);
        for(const auto & row2:rows){
            double cost=0;
            for(auto&agent :row_dict[row]){
                cost=std::max(cost,fabs(agent->current->x-row2));  
            }
            cost_matrix.back().push_back(cost);
        }
    }
    std::vector<int> assignment;
    labp_solve(cost_matrix,assignment);
    // std::cout<<"solved"<<std::endl;
    for(int i=0;i<rows.size();i++){
        for(auto&agent: row_dict[rows[i]]){
            agent->intermediate=graph->getVertex(rows[assignment[i]],agent->intermediate->y,agent->intermediate->z);
           
        //    std::cout<< agent->intermediate->print()<<std::endl;
        }
    }
    
}


/**
 * @brief 
 * shuffle robots to the intermediate states
 */
void RTH_2d::y_shuffle(){
    for(int i=0;i<graph->xmax;i+=cell_size){
        Robots robots1;
        int zs;
        for(auto &r:robots){
            if(r->current->x<cell_size+i and r->current->x>=i){
                robots1.push_back(r);
                zs=r->current->z;
            }
                
        }
        Motion3d swapper(robots1,{i,i+cell_size-1},{0,graph->ymax-1},{zs,zs},'y',graph);
        swapper.reconfigure();
    }
}

void RTH_2d::x_shuffle(){
    for(int i=0;i<graph->ymax;i+=cell_size){
        Robots robots1;
        int zs;
        for(auto &r:robots){
            if(r->current->y<cell_size+i and r->current->y>=i){
                robots1.push_back(r);
                zs=r->current->z;
            }
                
        }
        Motion3d swapper(robots1,{0,graph->xmax-1},{i,i+cell_size-1},{zs,zs},'x',graph);
        swapper.reconfigure();
    }
    
   
}

void RTH_2d::x_fitting(){
    for(auto &r:robots){
        r->intermediate=getV(r->inter_goal->x,r->current->y,r->current->z);
    }
}

void RTH_2d::y_fitting(){
    for(auto &r:robots){
        r->intermediate=getV(r->current->x,r->inter_goal->y,r->current->z);
    }
}