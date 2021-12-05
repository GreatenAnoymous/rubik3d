/**
 * @file rth2d.h
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2021-12-04
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#pragma once

#include"common.hpp"
#include"mp3d.hpp"



/**
 * @brief 
 * using 2d 3m shuffles for the fat columns
 */
class RTH_2d{
  
    using point2d=std::pair<int,int>;
public:
    RTH_2d(Robots &,Grids3d *graph);
    void solve();

private:
    // std::unordered_map<int,Location3d*> inter_goal2d;   //the desired (x,y)
    Location3d * getV(int,int,int);
    void matching();
    void x_shuffle();
    void x_fitting();
    void y_fitting();
    void y_shuffle();
    void prepare();
    void matching_helper(
        std::unordered_map<int,Robots> &column_dict,
        std::unordered_map<int,int> &matching,
        std::unordered_map<point2d,Robot*,boost::hash<point2d>> &arranged_robots,
        int row
    );
    void LBA_heuristic();
    Robots robots;// all the robots currently have the same z 
    Grids3d *graph;
};