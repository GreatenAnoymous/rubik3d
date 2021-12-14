/**
 * @file formation.cpp
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2021-12-14
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#include "formation.hpp"
#include "labp.hpp"
#include "search.hpp"
// #include <ortools/graph/ebert_graph.h>
// #include <ortools/graph/linear_assignment.h>

// using namespace operations_research;

FormationControl::FormationControl(Configs &starts,Configs &goals,Grids3d *graph):
    starts(starts),goals(goals),graph(graph){
    
}

FormationControl::FormationControl(Robots &robots,Grids3d *graph){
    for(auto &r:robots){
        starts.push_back(r->start);
        goals.push_back(r->goal);
    }
    this->graph=graph;
}

/**
 * @brief using BFS to find the initial paths
 * 
 * @param paths 
 */
void FormationControl::find_initial_paths(Paths3d & paths){
    paths.clear();
    using costMatrix=std::vector<std::vector<double>>;
    auto m=starts.size();
    std::vector<int> rowsol(m);
    std::vector<int> colsol(m);
    std::vector<double> u(m);
    std::vector<double> v(m);
   
    int threshold=20;        //if there is a lap algorithm for sparse matrix, that would be great
    bool feasible=false;
    costMatrix costs(m,std::vector<double>(m,BIG));
    for(int i=0;i<m;i++){
        for(int j=0;j<m;j++){
            double dist=starts[i]->manhattan_dist(goals[j]);
            if(dist<threshold) costs[i][j]=dist;
        }
    }
    lap(m,costs,rowsol,colsol,u,v);
    Configs new_goals;
    for(int i=0;i<m;i++){
        new_goals.push_back(goals[rowsol[i]]);
    }
    goals.swap(new_goals);
    for(int i=0;i<m;i++){
        BFS_solver solver(starts[i]);
        solver.isGoal=[&](Location3d *v){
            return v==goals[i];
        };
        solver.getNeighbors=[&](Location3d*v){
            return graph->getNeighbors(v);
        };
        auto pi=solver.solve();
        paths.push_back(pi);
    }

}

/**
 * @brief 
 * 
 * @param old_paths 
 */

void FormationControl::update_paths(Paths3d &old_paths){
    std::cout<<"/////////////////////////updating/////////////////////\n";
    int num_agents=old_paths.size();
    auto toPathSet=[](Path3d &p){
        std::unordered_set<Location3d*> path_set;
        for(auto &vs:p) path_set.insert(vs);
        return path_set;
    };
    using LocationSet=std::unordered_set<Location3d*>;
    LocationSet goalSet,startSet;
    std::unordered_map<int,int> degrees;

    auto findStandAloneGoal=[&](){
        for(auto &goal:goalSet){
            if(degrees[goal->id]<=1) return goal;
        }
        throw std::runtime_error("no standlone goal!");
    };

    for(int i=0;i<num_agents;i++){
        goalSet.insert(old_paths[i].back());
        startSet.insert(old_paths[i][0]);
        degrees[old_paths[i].back()->id]=0;
    }

    std::vector<LocationSet> path_sets;
    for(int i=0;i<num_agents;i++){
        LocationSet pi=toPathSet(old_paths[i]);
        path_sets.push_back(pi);
    }

    for(int i=0;i<num_agents;i++){
        for(auto &v:path_sets[i]) if(degrees.find(v->id)!=degrees.end()) degrees[v->id]++;
    }

    path_sets.clear();
    DAG dag;
    formDAG(old_paths,dag);
    Paths3d new_paths;
    while(not startSet.empty()){
        auto standAloneGoal=findStandAloneGoal();
        BFS_solver searcher(standAloneGoal);
        searcher.getNeighbors=[&](Location3d *v){
            auto possible_n=dag[v->id];
            Configs tmp;
            for(auto &n:possible_n){
                if(n.second>0) tmp.push_back(graph->getVertex(n.first));
            }
            return tmp;
        };
        searcher.isGoal=[&](Location3d*v){
            return startSet.find(v)!=startSet.end();
        };
        Path3d pi=searcher.solve();

        assert( not pi.empty());
        auto si=pi.back();
        startSet.erase(si);
        goalSet.erase(standAloneGoal);
        std::reverse(pi.begin(),pi.end());
        new_paths.push_back(pi);
        degrees.erase(standAloneGoal->id);
        auto new_pathSet=toPathSet(pi);

        //remove the weight of used edges
        for(int t=pi.size()-1;t>0;t--){
            auto it1=dag[pi[t]->id].begin();
            auto it2=dag[pi[t]->id].end();
            auto cond=[&](point2d &v){
                return v.first==pi[t-1]->id;
            };
            auto it=std::find_if(it1,it2,cond);
            it->second=it->second-1;
        }

        for(auto &v:new_pathSet){
            if(degrees.find(v->id)!=degrees.end()) degrees[v->id]--;
        }
    }
    std::cout<<"updated : num_agents="<<new_paths.size()<<std::endl;
    old_paths.swap(new_paths);    
}

/**
 * @brief schedule the paths to time parametrized
 * 
 * @param old_paths 
 */

void FormationControl::schedule(Paths3d &old_paths){
    auto num_agents=old_paths.size();
    using timeObstacle=std::tuple<int,int>;
    Paths3d timed_paths;
    std::unordered_set<timeObstacle,boost::hash<timeObstacle>> reserveTable;
    for(auto &p:old_paths){
        Path3d pi;
        int t=0,k=0;
        while(k<p.size()){
            timeObstacle obsk={p[k]->id,t};
            if(reserveTable.find(obsk)==reserveTable.end()){
                pi.push_back(p[k]);
                reserveTable.insert(obsk);
                t++;
                k++;
            }else{
                //wait
                pi.push_back(pi.back());
                auto v=pi.back();
                reserveTable.insert({v->id,t});
                t++;
            }          
        }  
        timed_paths.push_back(pi);
    }
    old_paths.swap(timed_paths);
}

/**
 * @brief 
 * 
 * @param paths 
 * @param dag_graph 
 */
void FormationControl::formDAG(const Paths3d &paths,DAG&dag_graph){
    for(auto &p:paths){
        for(int t=p.size()-1;t>=1;t--){
            auto it1=dag_graph[p[t]->id].begin();
            auto it2=dag_graph[p[t]->id].end();
            auto cond=[&](point2d & v){
                return v.first==p[t-1]->id;
            };
            auto it=std::find_if(it1,it2,cond);
            if(it==it2) dag_graph[p[t]->id].push_back({p[t-1]->id,1});
            else it->second=it->second+1;
            
        }
    }
}


void FormationControl::solve(){
    find_initial_paths(result);
    update_paths(result);
    schedule(result);
}


Paths3d FormationControl::get_result(){
    return result;
}