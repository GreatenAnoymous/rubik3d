/**
 * @file search.hpp
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2021-12-10
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#include "common.hpp"
#include <queue>


/**
 * @brief A star
 * 
 */

class AStarSolver{
public:
    struct AStarNode
    {
        using AStarNode_p=std::shared_ptr<AStarNode>;
        AStarNode(Location3d* v,int t,int f,int hc,AStarNode_p parent=nullptr):v(v),t(t),f(f),hc(hc),parent(parent){}
        int t;
        Location3d* v;
        int f;
        int hc; //number of conflicts; used for tie-breaking
        AStarNode_p parent;
        std::string toString(){
            return std::to_string(v->id)+"_"+std::to_string(t);
        }
    };

    using AStarNode_p=std::shared_ptr<AStarNode>;

    using AStarNodes=std::vector<AStarNode_p>;
    AStarSolver(Location3d *start,Location3d *goal);
    Path3d search();
    std::function<bool(AStarNode_p)> checkValid;
    std::function<AStarNodes(AStarNode_p)> getNeighbors;
    std::function<bool(AStarNode_p)> isSolution;
    std::function<int(AStarNode_p)> computeConflicts; 
    std::function<int(AStarNode_p)> computeHeuristic;
    std::function<bool(AStarNode_p,AStarNode_p)> compareOpen;
    using openList=std::priority_queue<AStarNode_p,AStarNodes,decltype(compareOpen)>;

    void standard_init();
  
private:
    Location3d* start,*goal;
};


/**
 * @brief  BFS
 * 
 */
class BFS_solver{
public:
    BFS_solver(Location3d *v):start(v){}
    std::function<bool(Location3d *v)> isGoal;
    std::function<Configs(Location3d *)> getNeighbors;
    using openList=std::queue<Location3d*>;
    Path3d solve();
   
private:
    Location3d *start;
    int depth;
};
