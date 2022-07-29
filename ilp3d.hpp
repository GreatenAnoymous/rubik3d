/**
 * @file ilp3d.hpp
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2021-12-29
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#ifndef ILP3D_HPP
#define ILP3D_HPP

#include"common.hpp"
#include "gurobi_c++.h"

static GRBEnv genv;

class Ilp3D{
public :
    Ilp3D(Configs &starts,Configs &goals,Grids3d *graph);
    GRBEnv *envp = &genv;
    typedef std::string id_type;
    void solve_original();
    void solve_split(Configs s,Configs g,int split,Paths3d &result);
    Paths3d get_result();
    void save_result(std::string file_name,double comp_time);
    int makespan;

protected:
    Configs starts,goals;
    Grids3d *graph;
    GRBModel prepare_model(int time_steps);
    void retrive_paths(GRBModel &model, int time_steps);
    void retrive_paths(GRBModel &model, size_t time_steps);
    inline id_type get_id(size_t r, Location3d* v1, Location3d* v2, size_t t);
    inline void store_var_for_vertices(GRBVar &var, Location3d* v1, Location3d* v2, size_t t1, size_t t2);
    inline void store_var_for_robots(GRBVar &var, size_t r, Location3d* v1, Location3d* v2, size_t t1, size_t t2);
    inline void store_var_for_edges(GRBVar &var, Location3d* v1, Location3d* v2, size_t t);

    std::map<id_type, GRBVar> edge_var_map = std::map<id_type, GRBVar>();
    std::map<id_type, std::vector<GRBVar>> edge_time_vector_map = std::map<id_type, std::vector<GRBVar>>();
    std::map<id_type, std::vector<GRBVar>> time_in_vector_map = std::map<id_type, std::vector<GRBVar>>();
    std::map<id_type, std::vector<GRBVar>> time_out_vector_map = std::map<id_type, std::vector<GRBVar>>();
    std::map<id_type, std::vector<GRBVar>> robot_in_vector_map = std::map<id_type, std::vector<GRBVar>>();
    std::map<id_type, std::vector<GRBVar>> robot_out_vector_map = std::map<id_type, std::vector<GRBVar>>();
    std::vector<std::set<size_t>> reachability = std::vector<std::set<size_t>>();
    std::vector<std::vector<size_t>> individual_paths = std::vector<std::vector<size_t>>();
    Paths3d final_paths;

    
};

void ilp_solve_split(Configs &starts,Configs &goals,Grids3d *graph,int split,int &makespan);
void get_middle_configs(Configs &starts,Configs &goals, Grids3d *graph,Configs &mid);
void save_makespan_and_time(std::string file_name,int makespan,int makespanLB,double runtime);
#endif
