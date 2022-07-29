/**
 * @file RTH3d_obs.hpp
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2022-01-30
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#pragma once
#include"rubik3d.hpp"


class RTH_3d_obs:public RTH_3d{
public:
    RTH_3d_obs(Robots &robots, Grids3d *graph):RTH_3d(robots,graph){
        // printf("Debug grid xmax,ymax,zmax=%d,%d,%d\n",graph->xmax,graph->ymax,graph->zmax);
    }
    void solve();
    void lba_matching_fat();
    
protected:
    void random_to_balanced_fast();
    void z_shuffle();
    // void lba_matching_fat();

};



