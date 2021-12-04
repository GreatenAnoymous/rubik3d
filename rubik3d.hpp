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
#include "rth2d.hpp"
#include "mp3d.hpp"


/**
 * @brief 
 * 3d shuffle schemes
 */

class RTH_3d{
using point3d=std::tuple<int,int,int>;
using robotPlaceMap=std::unordered_map<point3d,Robots,boost::hash<point3d>>;
public:
    RTH_3d(){}
    RTH_3d(Robots &, Grids3d *graph);
    void solve();

protected:
    Robots robots;
    Grids3d * graph;
    void matching_heuristic();
    void xy_shuffle();
    void z_shuffle();
    void xy_fitting();
    void z_fitting(); 
    void lba_matching_fat();
    // void lba_matching_2d();
    void prepare();
    std::function<Location3d*(int,int,int)> getVertex;
    void prepare_helper(int i,int j,int k,robotPlaceMap & ,robotPlaceMap &);
    
};




