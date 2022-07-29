/**
 * @file formation.hpp
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2021-12-14
 * 
 * Implementation of distance-optimal UMAPF on 3d grids
 * "Distance optimal formation control on graphs with a tight convergence time guarantee." 2012 IEEE 51st IEEE Conference on Decision and Control (CDC). IEEE, 2012.
 * Yu, Jingjin, and M. LaValle. 
 * @copyright Copyright (c) 2021
 * 
 */

#pragma once
#include "common.hpp"

class FormationControl{
public:
    using point2d=std::pair<int,int>;   //(vertex, weight)
    using DAG=std::unordered_map<int,std::vector<point2d>>;
    FormationControl(Configs &starts,Configs &goals,Grids3d *graph);
    FormationControl(Robots &robots,Grids3d *graph);
    void solve();
    Paths3d get_result();

private:
    Configs starts;
    Configs goals;
    Grids3d* graph;
    Paths3d result;
    void find_initial_paths(Paths3d & paths);
  
    void update_paths(Paths3d &paths);
    void schedule(Paths3d &);
    void formDAG(const Paths3d &,DAG &dag_graph);
};
