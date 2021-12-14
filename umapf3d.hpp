/**
 * @file umapf3d.hpp
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2021-12-04
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#pragma once
#include "common.hpp"


/**
 * @brief 
 * anonymous MAPF solver 
 */
class UMAPF3d{
public:
    UMAPF3d(Configs&_s,Configs&_g,Grids3d *_graph):starts(_s),goals(_g),graph(_graph){}
    void solve();
    Paths3d get_result(){
        return result;
    }

private:
    Configs starts,goals;
    Grids3d *graph;
    Paths3d result;
    double runtime;

};




