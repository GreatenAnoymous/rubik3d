/**
 * @file search.cpp
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2021-12-10
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#include "search.hpp"

///////////////BFS/////////////////////////////////////////////////////

Path3d BFS_solver::solve(){
    openList open;
    open.push(start);
    std::unordered_set<Location3d*> closed;
    std::unordered_map<Location3d*,Location3d*> parents;
    bool success=false;
    Location3d* curr;
    while(open.empty()==false){
        auto top=open.front();
        open.pop();
        if(isGoal(top)){
            success=true;
            curr=top;
     
            break;
        }
        closed.insert(top);
        auto children=getNeighbors(top);
        for(auto &c:children){
            if(closed.find(c)!=closed.end()) continue;
            parents[c]=top;
            open.push(c);
        }

    }
    //retrieve the path
    if(success==false) return {};
    Path3d result;
    while(curr!=start){
        result.push_back(curr);
        curr=parents[curr];
    }
    result.push_back(start);
    std::reverse(result.begin(),result.end());
    return result;  
}

//////////////////////////////

AStarSolver::AStarSolver(Location3d*start,Location3d*goal){
    this->start=start;
    this->goal=goal;
}

Path3d AStarSolver::search(){
    openList open(compareOpen);
    std::unordered_set<std::string> closed;
    int f0,g0,fLB=0;
    f0=start->manhattan_dist(goal);
    AStarNode_p start_node=std::make_shared<AStarNode>(start,f0,0,0,nullptr);
    open.push(start_node);
    bool success=false;
    AStarNode_p goalNode;
    while(open.empty()==false){
        auto best=open.top();
        open.pop();
        if(isSolution(best)){
            success=true;
            goalNode=best;
            break;
        }

        
        if(closed.find(best->toString())!=closed.end()) continue;
        closed.insert(best->toString());
 
        auto neighbors=getNeighbors(best);
        // auto neighbors=best->v->neighbor;
        for(auto &n :neighbors){
            // std::cout<<"child "<<n->toString()<<" ";
            
            if(closed.find(n->toString())!=closed.end()) continue;           // not closed
 
            open.push(n);                                            //push to the open
        }
    }
    if(success==false){

        return {};              //fail to find a solution
    }
    else{
        //extract the path
        // std::cout<<"found! number of expansions="<<numberOfExpansions<<std::endl;
        AStarNode_p current=goalNode;
        Path3d result;
        while(true){
            // current->v->println();
            result.push_back(current->v);
            if(current==start_node) break;
            current=current->parent;
        }
        std::reverse(result.begin(),result.end());
        //shrink
        return result;
     
    }
}

void AStarSolver::standard_init(){
    compareOpen=[&](AStarNode_p a1, AStarNode_p a2)->bool{
        if(a1->f!=a2->f) return a1->f> a2->f;
        // if(a1->t!=a2->t) return a1->t< a2->t;
        return false;
    };
    isSolution=[&](AStarNode_p n)->bool{
        // std::cout<<"max_constraint_time= "<<max_constraint_time<<std::endl;
        return n->v==this->goal;
    };
    // std::cout<<"function init complete!"<<std::endl;
}
