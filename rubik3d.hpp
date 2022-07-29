#pragma once
/**
 * @file rubik3d.h
 * @author Greaten (2274880117@qq.com)
 * @brief 
 * @version 0.1
 * @date 2021-11-24
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#include"common.hpp"



/**
 * @brief 
 * 3d shuffle schemes
 */

class RTH_3d{

public:
using point3d=std::tuple<int,int,int>;
using point2d=std::pair<int,int>;
using robotPlaceMap=std::unordered_map<point3d,Robots,boost::hash<point3d>>;
    RTH_3d(){}
    RTH_3d(Robots &, Grids3d *graph);
    void solve();
    void save_result(std::string out_name,double runtime=0,bool save_paths=false){
        remove_virtual_robots(robots);
        save_solutions(out_name,robots,runtime,save_paths);
    } 

protected:
    Robots robots;
    Grids3d * graph;
    void matching_heuristic();
    void xy_shuffle();
    void z_shuffle();
    void xy_fitting();
    void z_fitting(); 
    void lba_matching_fat();
    void matching_helper(std::unordered_map<int,Robots> &column_dict,
        std::unordered_map<int,int> &matching,
        std::unordered_map<point2d,Robot*,boost::hash<point2d>> &arranged_agents,
        int row);

    void prepare();
    std::function<Location3d*(int,int,int)> getVertex;
    void prepare_helper(int i,int j,int k,robotPlaceMap & ,robotPlaceMap &);
    bool use_umapf=false;
    void LBA_heuristic();
    int get_plane_id(int x,int y){
        if(graph->xmax>=graph->ymax) return x+graph->xmax*y;
        else return y+graph->ymax*x;
    }
    void get_xy(int id,int &x,int &y){
        if(graph->xmax>=graph->ymax){
            x=id%(graph->xmax);
            y=id/(graph->xmax);
            // printf("debug get xy=%d,%d   xmax=%d,ymax=%d\n",x,y,graph->xmax,graph->ymax);
        }else{
            y=id%(graph->ymax);
            x=id/(graph->ymax);
            //  printf("debug get xy=%d,%d   xmax=%d,ymax=%d\n",x,y,graph->xmax,graph->ymax);
        }
    
    }
    void append_umapf_path();
    void random_to_balanced();
    void random_to_balanced_fast();
};




